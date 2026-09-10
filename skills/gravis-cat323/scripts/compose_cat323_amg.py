#!/usr/bin/env python3
"""Run on Orin: validate the optional test overlay or start an already-stopped AMG."""

import argparse
import copy
import grp
import hashlib
import json
import os
from pathlib import Path
import subprocess


CONTAINER_BUNDLE = "/run/cat323-map-transport"
NATIVE_LAUNCH = "/amg/install/lib/python3.12/site-packages/autonomy_visualization/launch.py"


def check_scope(normal, test, bundle, image_command=None):
    """Require an unchanged stack except for the two test mounts and AMG command."""
    baseline = copy.deepcopy(normal)
    candidate = copy.deepcopy(test)
    original_amg = baseline["services"]["amg"]
    test_amg = candidate["services"]["amg"]
    expected_command = ["python3", f"{CONTAINER_BUNDLE}/launch_amg.py"]
    if test_amg.get("command") != expected_command:
        raise RuntimeError("Unexpected test AMG command")
    # The observed normal command has no extra arguments. Do not silently drop future ones.
    normal_command = original_amg.get("command") or image_command
    if normal_command != ["ros2", "launch", "amg", "amg.launch.py"]:
        raise RuntimeError("Native AMG command changed; preserve its arguments explicitly before using this overlay")
    if "command" in original_amg:
        test_amg["command"] = original_amg["command"]
    else:
        test_amg.pop("command")
    original_mounts = {v["target"]: v for v in original_amg.get("volumes", [])}
    test_mounts = {v["target"]: v for v in test_amg.get("volumes", [])}
    for target, source in ((CONTAINER_BUNDLE, str(bundle)), (NATIVE_LAUNCH, str(bundle / "patched-launch.py"))):
        if target in original_mounts:
            raise RuntimeError(f"Native configuration already mounts {target}; review the collision")
        mount = test_mounts.pop(target, None)
        if not mount or mount.get("type") != "bind" or mount.get("source") != source or not mount.get("read_only"):
            raise RuntimeError(f"Unexpected test mount for {target}")
        # Compose's normalized JSON omits false booleans (the input sets false explicitly).
        if mount.get("bind", {}).get("create_host_path", False):
            raise RuntimeError(f"Test bind must not create a missing host path: {target}")
    if test_mounts != original_mounts:
        raise RuntimeError("Test overlay changes other AMG mounts")
    if "volumes" in original_amg:
        test_amg["volumes"] = original_amg["volumes"]
    else:
        test_amg.pop("volumes", None)
    if candidate != baseline:
        raise RuntimeError("Test overlay changes additional native configuration")
    return {"normal_command": normal_command, "test_command": expected_command, "only_test_command_and_two_readonly_mounts_differ": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("check", "start-test"))
    args = parser.parse_args()
    bundle = Path(__file__).resolve().parent
    manifest = json.loads((bundle / "manifest.json").read_text())
    native = manifest.get("native_deployment")
    if not native:
        raise RuntimeError("Prepare this bundle from the real native deployment before using Compose")
    base_files = [Path(p) for p in native["compose_files"]]
    config_dir = Path(native["compose_directory"])
    for path in base_files:
        if not path.is_file():
            raise RuntimeError(f"Run this bundle on the Orin host; missing native Compose file: {path}")
    for name, field in (("patched-launch.py", "patched_launch_sha256"), ("selected-map-fastdds.xml", "profile_sha256")):
        if hashlib.sha256((bundle / name).read_bytes()).hexdigest() != manifest[field]:
            raise RuntimeError(f"Bundle file changed after preparation: {name}")
    environment = os.environ.copy()
    environment["CAT323_MAP_TRANSPORT_BUNDLE"] = str(bundle)
    # These are the same group/secret-path inputs used by the installed Gravis Taskfile.
    # Pin the image used to prepare this bundle; do not silently switch the native build.
    environment["AMG_BASE_IMAGE"] = native["image"]
    environment["GRAVIS_GID"] = str(grp.getgrnam("gravis").gr_gid)
    environment["RENDER_GID"] = str(grp.getgrnam("render").gr_gid)
    secret_path = Path("/etc/gravisrobotics/secrets/septentrio_password")
    environment["SEPTENTRIO_PASSWORD_SECRET_FILE"] = str(secret_path) if secret_path.is_file() else ""
    normal = ["docker", "compose", "--project-directory", str(config_dir), "-p", native["compose_project"]]
    for path in base_files:
        normal.extend(["-f", str(path)])
    test = normal + ["-f", str(bundle / "docker-compose.map-test.yaml")]

    def config(command):
        result = subprocess.run(command + ["config", "--format", "json"], env=environment, capture_output=True, text=True, timeout=25)
        if result.returncode:
            raise RuntimeError("Native Compose configuration failed: " + result.stderr.strip())
        return json.loads(result.stdout)

    base_config = config(normal)
    base_service = base_config["services"]["amg"]
    base_mounts = [{"source": m["source"], "target": m["target"], "read_only": m.get("read_only", False)} for m in base_service.get("volumes", []) if m["type"] == "bind"]
    for secret in base_service.get("secrets", []):
        target = secret.get("target", secret["source"])
        if not target.startswith("/"):
            target = "/run/secrets/" + target
        base_mounts.append({"source": base_config["secrets"][secret["source"]]["file"], "target": target, "read_only": True})

    def normalize(mounts):
        return sorted([{**m, "source": str(Path(m["source"]).resolve())} for m in mounts], key=lambda m: m["target"])

    if normalize(base_mounts) != normalize(native["bind_mounts"]):
        raise RuntimeError("Native bind mounts differ from the captured deployment; check CONFIG_REPO_ROOT and invoking user")
    image_id = subprocess.run(["docker", "image", "inspect", "--format", "{{.Id}}", native["image"]], capture_output=True, text=True, timeout=10, check=True).stdout.strip()
    if image_id != native["image_id"]:
        raise RuntimeError("Native image tag now points at a different image; prepare a new test bundle")
    report = check_scope(base_config, config(test), bundle, native["command"])
    if args.action == "check":
        print(json.dumps({"action": "check", "started": False, **report}, indent=2))
        return
    owners = subprocess.run(["docker", "ps", "-q", "--filter", "label=com.docker.compose.project=" + native["compose_project"], "--filter", "label=com.docker.compose.service=amg"], capture_output=True, text=True, timeout=10, check=True)
    if owners.stdout.strip():
        raise RuntimeError("An AMG Compose generation is running; use the normal Gravis stop procedure before switching mode")
    inspect = subprocess.run(["docker", "inspect", "--format", "{{.State.Running}}", "amg"], capture_output=True, text=True, timeout=10)
    if inspect.returncode:
        if "No such object" not in inspect.stderr and "No such container" not in inspect.stderr:
            raise RuntimeError("Cannot establish AMG container state; no start attempted")
    elif inspect.stdout.strip() != "false":
        raise RuntimeError("AMG is running; stop it through the normal Gravis procedure before switching mode")
    # Keep the installed image and other native services. Never stop/restart a running AMG here.
    subprocess.run(test + ["up", "-d", "--no-deps", "--no-build", "--pull", "never", "amg"], env=environment, check=True)


if __name__ == "__main__":
    main()
