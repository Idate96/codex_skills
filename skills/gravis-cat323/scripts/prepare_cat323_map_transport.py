#!/usr/bin/env python3
"""Prepare a local, opt-in AMG launch bundle without changing the running machine."""

import argparse
import ast
import difflib
import hashlib
import json
import re
from pathlib import Path
import shlex
import subprocess


LAUNCH_PATH = "/amg/install/lib/python3.12/site-packages/autonomy_visualization/launch.py"
LAUNCH_FUNCTION = "get_gridmap_visualization_launch_actions"
BASE_PROFILE = "/etc/gravisrobotics/fastdds/client_configuration.xml"
NODE_NAME = "grid_map_selected_layers_interface_republisher_node"
PROFILE_ENV = "CAT323_SELECTED_MAP_DDS_PROFILE"
SKILL = Path(__file__).resolve().parent.parent


def digest(data):
    return hashlib.sha256(data).hexdigest()


def patch_launch(source):
    """Keep all native actions, adding an opt-in environment to one known Node."""
    if PROFILE_ENV in source:
        raise ValueError("Launch already has CAT323 DDS integration; review it instead of patching again")
    tree = ast.parse(source)
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == LAUNCH_FUNCTION]
    if len(functions) != 1:
        raise ValueError(f"Require one {LAUNCH_FUNCTION} function")
    calls = []
    for node in ast.walk(functions[0]):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name) or node.func.id != "Node":
            continue
        keywords = {k.arg: k.value for k in node.keywords}
        name = keywords.get("name")
        if isinstance(name, ast.Constant) and name.value == NODE_NAME:
            calls.append(node)
    if len(calls) != 1:
        raise ValueError("Require exactly one selected-interface Node in the launch")
    call = calls[0]
    keywords = {k.arg: k.value for k in call.keywords}
    expected = {"package": "autonomy_visualization", "namespace": "grid_map_postprocessed_interface", "executable": "gridmap_selected_layers_republisher_node"}
    for key, value in expected.items():
        node = keywords.get(key)
        if not isinstance(node, ast.Constant) or node.value != value:
            raise ValueError(f"Selected Node's {key} changed; review the new native launch")
    if "env" in keywords or "additional_env" in keywords or None in keywords:
        raise ValueError("Selected Node already has environment/kwargs customization; merge explicitly")
    lines = source.splitlines(keepends=True)
    name_keyword = next(k for k in call.keywords if k.arg == "name")
    name_line = lines[name_keyword.lineno - 1]
    if not name_line.lstrip().startswith("name=") or name_keyword.end_lineno != name_keyword.lineno:
        raise ValueError("Unexpected Node formatting; review before patching")
    indent = name_line[:len(name_line) - len(name_line.lstrip())]
    block = (
        "    # CAT323 selected-map DDS override (opt-in).\n"
        f"    cat323_profile = os.environ.get({PROFILE_ENV!r}, '')\n"
        "    cat323_map_env = {}\n"
        "    if cat323_profile:\n"
        "        if not os.path.isfile(cat323_profile):\n"
        "            raise RuntimeError(f'CAT323 selected-map DDS profile missing: {cat323_profile}')\n"
        "        cat323_map_env = {\n"
        "            'FASTRTPS_DEFAULT_PROFILES_FILE': cat323_profile,\n"
        "            'RMW_FASTRTPS_USE_QOS_FROM_XML': '1',\n"
        "        }\n\n"
    )
    edits = [(name_keyword.end_lineno, indent + "additional_env=cat323_map_env,\n")]
    first_statement = functions[0].body[0]
    if isinstance(first_statement, ast.Expr) and isinstance(first_statement.value, ast.Constant) and isinstance(first_statement.value.value, str):
        edits.append((first_statement.end_lineno, block))
    else:
        edits.append((first_statement.lineno - 1, block))
    has_os = any(isinstance(n, ast.Import) and any(a.name == "os" and a.asname in (None, "os") for a in n.names) for n in tree.body)
    if not has_os:
        imports = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom)) and not (isinstance(n, ast.ImportFrom) and n.module == "__future__")]
        if not imports:
            raise ValueError("Native launch has no recognized imports")
        edits.append((imports[0].lineno - 1, "import os\n\n"))
    for index, text in sorted(edits, reverse=True):
        lines.insert(index, text)
    patched = "".join(lines)
    ast.parse(patched)
    return patched


def read_native(host, container):
    script = f"""import hashlib,json
from pathlib import Path
launch=Path({LAUNCH_PATH!r}).read_text()
profile=Path({BASE_PROFILE!r}).read_bytes()
print(json.dumps({{'launch':launch,'base_profile_sha256':hashlib.sha256(profile).hexdigest()}}))
"""
    host_script = f"""import json,subprocess
native=json.loads(subprocess.run(['docker','exec',{container!r},'python3','-c',{script!r}],capture_output=True,text=True,check=True).stdout)
info=json.loads(subprocess.run(['docker','inspect',{container!r}],capture_output=True,text=True,check=True).stdout)[0]
labels=info['Config']['Labels']
native['metadata']={{'image':info['Config']['Image'],'image_id':info['Image'],'image_user':info['Config'].get('User',''),'command':info['Config']['Cmd'],'compose_files':labels['com.docker.compose.project.config_files'].split(','),'compose_directory':labels['com.docker.compose.project.working_dir'],'compose_project':labels['com.docker.compose.project'],'bind_mounts':[{{'source':m['Source'],'target':m['Destination'],'read_only':not m['RW']}} for m in info['Mounts'] if m['Type']=='bind']}}
print(json.dumps(native))
"""
    command = shlex.join(["python3", "-"])
    result = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", host, command], input=host_script, text=True, capture_output=True, timeout=20, check=True)
    native = json.loads(result.stdout)
    native["metadata"]["bind_mounts"].sort(key=lambda mount: mount["target"])
    return native


def prepare(source, base_hash, output, metadata=None):
    patched = patch_launch(source)
    profile = (SKILL / "assets/cat323-selected-map-fastdds.xml").read_bytes()
    manifest = {
        "format": 1,
        "native_launch_path": LAUNCH_PATH,
        "base_profile_path": BASE_PROFILE,
        "base_profile_sha256": base_hash,
        "original_launch_sha256": digest(source.encode()),
        "patched_launch_sha256": digest(patched.encode()),
        "profile_sha256": digest(profile),
        "profile_environment_variable": PROFILE_ENV,
        "native_deployment": metadata,
    }
    patch = "".join(difflib.unified_diff(source.splitlines(keepends=True), patched.splitlines(keepends=True), fromfile="original/" + Path(LAUNCH_PATH).name, tofile="patched/" + Path(LAUNCH_PATH).name))
    files = {
        "original-launch.py": source.encode(),
        "patched-launch.py": patched.encode(),
        "selected-map-fastdds.xml": profile,
        "launch.patch": patch.encode(),
        "manifest.json": (json.dumps(manifest, indent=2) + "\n").encode(),
        "launch_amg.py": (SKILL / "scripts/launch_cat323_amg.py").read_bytes(),
        "compose_amg.py": (SKILL / "scripts/compose_cat323_amg.py").read_bytes(),
        "docker-compose.map-test.yaml": (SKILL / "assets/docker-compose.map-test.yaml").read_bytes(),
        "README.md": (SKILL / "references/map-transport-test.md").read_bytes(),
    }
    if metadata is not None:
        image_id = metadata.get("image_id", "")
        if not re.fullmatch(r"sha256:[0-9a-f]{64}", image_id):
            raise ValueError("Require the inspected immutable native image ID")
        image_user = metadata.get("image_user", "") or "root"
        if not re.fullmatch(r"[A-Za-z0-9_.:-]+", image_user):
            raise ValueError("Unsupported native image user")
        files["Dockerfile"] = (
            f"FROM {image_id}\n"
            f"COPY patched-launch.py {LAUNCH_PATH}\n"
            "COPY selected-map-fastdds.xml /amg/install/share/autonomy_visualization/config/selected-map-fastdds.xml\n"
            "USER root\n"
            "RUN rm -f /amg/install/lib/python3.12/site-packages/autonomy_visualization/__pycache__/launch.*.pyc\n"
            f"USER {image_user}\n"
        ).encode()
    for name, content in files.items():
        path = output / name
        if path.exists() and path.read_bytes() != content:
            raise ValueError(f"Refusing to overwrite different content: {path}; use another output directory")
    output.mkdir(parents=True, exist_ok=True)
    for name, content in files.items():
        path = output / name
        if not path.exists():
            path.write_bytes(content)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="orin-a")
    parser.add_argument("--container", default="amg")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--native-launch", type=Path, help="Use a local native launch copy instead of SSH")
    parser.add_argument("--base-profile", type=Path, help="Native profile copy, required with --native-launch")
    args = parser.parse_args()
    if bool(args.native_launch) != bool(args.base_profile):
        parser.error("--native-launch and --base-profile must be supplied together")
    if args.native_launch:
        native = {"launch": args.native_launch.read_text(), "base_profile_sha256": digest(args.base_profile.read_bytes())}
    else:
        native = read_native(args.host, args.container)
    manifest = prepare(native["launch"], native["base_profile_sha256"], args.output, native.get("metadata"))
    print(json.dumps({"output": str(args.output.resolve()), "remote_changes": False, **manifest}, indent=2))


if __name__ == "__main__":
    main()
