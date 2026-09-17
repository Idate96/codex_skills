#!/usr/bin/env python3
"""Offline checks for opt-in scope, drift rejection, and process ownership."""

import copy
import json
import os
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import cat323_map_transport_remote as remote
import check_cat323_maps as maps
import compose_cat323_amg as compose
import launch_cat323_amg as launcher
import prepare_cat323_map_transport as prepare


FIXTURE = '''from gravis_shared.launch import Node

def get_gridmap_visualization_launch_actions():
    first = Node(
        package="autonomy_visualization",
        namespace="grid_map_postprocessed",
        executable="gridmap_selected_layers_republisher_node",
        name="grid_map_selected_layers_republisher_node",
    )
    selected = Node(
        package="autonomy_visualization",
        namespace="grid_map_postprocessed_interface",
        executable="gridmap_selected_layers_republisher_node",
        name="grid_map_selected_layers_interface_republisher_node",
        parameters=["native.yaml"],
    )
    last = Node(
        package="autonomy_visualization",
        namespace="elevation_map_raw",
        executable="gridmap_selected_layers_republisher_node",
        name="elevation_map_selected_layers_republisher_node",
    )
    return [first, selected, last]
'''


def evaluate(source, profile=None):
    native = types.ModuleType("gravis_shared.launch")
    native.Node = lambda **kwargs: kwargs
    namespace = {}
    with patch.dict("sys.modules", {"gravis_shared.launch": native}), patch.dict(os.environ):
        os.environ.pop(prepare.PROFILE_ENV, None)
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = "native.xml"
        if profile is not None:
            os.environ[prepare.PROFILE_ENV] = profile
        exec(compile(source, "fixture.py", "exec"), namespace)
        result = namespace[prepare.LAUNCH_FUNCTION]()
        if os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] != "native.xml":
            raise AssertionError("Launch changed global DDS environment")
        return result


class TransportTests(unittest.TestCase):
    def test_map_check_requires_advancing_fresh_source_stamps(self):
        fresh = [{"stamp_ns": (i + 1) * 1_000_000_000, "receipt_s": i, "age_s": 1.3} for i in range(30)]
        self.assertTrue(maps.summarize(fresh, 30)["passed"])
        repeated = [{**r, "stamp_ns": 1_000_000_000} for r in fresh]
        self.assertFalse(maps.summarize(repeated, 30)["passed"])
        fresh[10]["age_s"] = 3.01
        self.assertFalse(maps.summarize(fresh, 30)["passed"])

    def test_fresh_publications_can_expire_between_deliveries(self):
        rows = [{"stamp_ns": (i + 1) * 1_000_000_000, "receipt_s": float(i), "age_s": 2.2} for i in range(30)]
        report = maps.summarize(rows, 30, end_s=30)
        self.assertEqual(report["invalid_or_stale_publications"], 0)
        self.assertFalse(report["passed"])
        self.assertAlmostEqual(report["max_held_source_age_s"], 3.2)
        rows = [{**row, "age_s": 1.3} for row in rows]
        self.assertTrue(maps.summarize(rows, 30, end_s=30)["passed"])
        self.assertFalse(maps.summarize(rows, 30, end_s=32)["passed"])

    def test_image_context_preserves_base_identity_and_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "bundle"
            metadata = {"image_id": "sha256:" + "a" * 64, "image_user": "ubuntu"}
            prepare.prepare(FIXTURE, "base", output, metadata)
            dockerfile = (output / "Dockerfile").read_text()
            self.assertIn("FROM " + metadata["image_id"], dockerfile)
            self.assertIn("__pycache__/launch.*.pyc", dockerfile)
            self.assertTrue(dockerfile.endswith("USER ubuntu\n"))
            with self.assertRaises(ValueError):
                prepare.prepare(FIXTURE, "base", output, {**metadata, "image_id": "mutable:latest"})
            self.assertEqual((output / "Dockerfile").read_text(), dockerfile)

    def test_default_keeps_native_nodes_and_parameters(self):
        original = evaluate(FIXTURE)
        patched = evaluate(prepare.patch_launch(FIXTURE))
        self.assertEqual(patched[1].pop("additional_env"), {})
        self.assertEqual(original, patched)

    def test_test_profile_changes_only_selected_environment(self):
        with tempfile.NamedTemporaryFile() as profile:
            patched = evaluate(prepare.patch_launch(FIXTURE), profile.name)
            original = evaluate(FIXTURE)
            self.assertEqual(patched[0], original[0])
            self.assertEqual(patched[2], original[2])
            self.assertEqual(patched[1].pop("additional_env"), {"FASTRTPS_DEFAULT_PROFILES_FILE": profile.name, "RMW_FASTRTPS_USE_QOS_FROM_XML": "1"})
            self.assertEqual(patched[1], original[1])

    def test_missing_profile_fails_before_launch(self):
        with self.assertRaisesRegex(RuntimeError, "profile missing"):
            evaluate(prepare.patch_launch(FIXTURE), "/no/such/cat323-profile.xml")

    def test_duplicate_or_customized_node_requires_review(self):
        duplicate = FIXTURE.replace('name="grid_map_selected_layers_republisher_node"', f'name="{prepare.NODE_NAME}"')
        customized = FIXTURE.replace('parameters=["native.yaml"],', 'parameters=["native.yaml"],\n        additional_env={"EXISTING": "value"},')
        for source in (duplicate, customized, prepare.patch_launch(FIXTURE)):
            with self.assertRaises(ValueError):
                prepare.patch_launch(source)

    def test_bundle_is_repeatable_and_preserves_edits(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "bundle"
            prepare.prepare(FIXTURE, "base-hash", output)
            before = {p.name: p.read_bytes() for p in output.iterdir()}
            prepare.prepare(FIXTURE, "base-hash", output)
            self.assertEqual(before, {p.name: p.read_bytes() for p in output.iterdir()})
            (output / "patched-launch.py").write_text("operator edit")
            with self.assertRaisesRegex(ValueError, "Refusing to overwrite"):
                prepare.prepare(FIXTURE, "base-hash", output)
            self.assertEqual((output / "patched-launch.py").read_text(), "operator edit")

    def test_launch_refuses_changed_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("launch.py", "base.xml", "selected-map-fastdds.xml"):
                (root / name).write_text(name)
            manifest = {
                "native_launch_path": str(root / "launch.py"),
                "patched_launch_sha256": launcher.sha(root / "launch.py"),
                "base_profile_path": str(root / "base.xml"),
                "base_profile_sha256": launcher.sha(root / "base.xml"),
                "profile_sha256": launcher.sha(root / "selected-map-fastdds.xml"),
            }
            launcher.check_bundle(root, manifest)
            (root / "selected-map-fastdds.xml").write_text("changed")
            with self.assertRaises(RuntimeError):
                launcher.check_bundle(root, manifest)

    def test_detects_running_amg_and_republisher(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for pid, argv in [(10, [b"python3", b"/opt/ros/jazzy/bin/ros2", b"launch", b"amg", b"amg.launch.py"]), (11, [remote.EXE, remote.NODE]), (12, [b"unrelated"] )]:
                (root / str(pid)).mkdir()
                (root / str(pid) / "cmdline").write_bytes(b"\0".join(argv))
            self.assertEqual(set(launcher.running_amg(root)), {10, 11})

    def test_compose_preserves_all_other_configuration(self):
        bundle = Path("/tmp/test-bundle")
        normal = {"services": {"amg": {"image": "native:1", "command": ["ros2", "launch", "amg", "amg.launch.py"], "environment": {"NATIVE_DDS": "unchanged"}, "volumes": []}, "discovery": {"image": "discovery:1"}}}
        candidate = copy.deepcopy(normal)
        amg = candidate["services"]["amg"]
        amg["command"] = ["python3", compose.CONTAINER_BUNDLE + "/launch_amg.py"]
        for target, source in [(compose.CONTAINER_BUNDLE, str(bundle)), (compose.NATIVE_LAUNCH, str(bundle / "patched-launch.py"))]:
            amg["volumes"].append({"type": "bind", "target": target, "source": source, "read_only": True, "bind": {"create_host_path": False}})
        compose.check_scope(normal, candidate, bundle)
        # Compose 2.27 omits explicit false values in normalized JSON.
        for mount in amg["volumes"]:
            mount["bind"].pop("create_host_path")
        compose.check_scope(normal, candidate, bundle)
        amg["volumes"][0]["bind"]["create_host_path"] = True
        with self.assertRaisesRegex(RuntimeError, "must not create"):
            compose.check_scope(normal, candidate, bundle)
        amg["volumes"][0]["bind"].pop("create_host_path")
        amg["environment"]["NATIVE_DDS"] = "global override"
        with self.assertRaisesRegex(RuntimeError, "additional native configuration"):
            compose.check_scope(normal, candidate, bundle)

    def test_status_does_not_export_unrelated_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = root / "profile.xml"
            profile.write_bytes((prepare.SKILL / "assets/cat323-selected-map-fastdds.xml").read_bytes())
            proc = root / "123"
            proc.mkdir()
            proc.joinpath("cmdline").write_bytes(b"\0".join([remote.EXE, remote.NODE]))
            proc.joinpath("environ").write_bytes(f"FASTRTPS_DEFAULT_PROFILES_FILE={profile}\0SECRET=never-export-this\0".encode())
            report = remote.status(root)
            self.assertEqual(report["selected_publisher_count"], 1)
            self.assertEqual(report["publishers"][0]["outgoing_message_size"], "1400")
            self.assertNotIn("never-export-this", json.dumps(report))

    def test_restore_rejects_unowned_process(self):
        with self.assertRaisesRegex(RuntimeError, "No verified temporary supervisor"):
            remote.restore({"selected_publisher_count": 1, "publishers": [{"pid": 12, "temporary_supervisor": {"identity_matches": False}}]})


if __name__ == "__main__":
    unittest.main()
