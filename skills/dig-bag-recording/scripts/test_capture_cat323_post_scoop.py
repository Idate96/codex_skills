"""Offline checks for scoop identity, freshness, and a complete MCAP round trip."""
import json
from pathlib import Path
import tempfile
import unittest

from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray

from capture_cat323_post_scoop import fresh_source, read_attempt, require_latest_attempt, save_and_verify_map


class PostScoopCaptureTests(unittest.TestCase):
    def test_explicit_aborted_attempt_keeps_result_and_rejects_ambiguous_receipt(self):
        receipt = {"events": [
            {"event": "goal_response", "accepted": True, "goal_id": "4c72602b93b44857a268492244aacefc",
             "wall_s": 100.0},
            {"event": "result", "status": 6, "success": False, "message": "source expired", "wall_s": 110.0},
        ]}
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / "scoop-10.json"
            path.write_text(json.dumps(receipt))
            raw, assignment = read_attempt(root, Path("scoop-10.json"))
            self.assertEqual(json.loads(raw), receipt)
            self.assertEqual(assignment["scoop_number"], 10)
            self.assertEqual(assignment["goal_id"], receipt["events"][0]["goal_id"])
            self.assertFalse(assignment["action_result"]["success"])
            self.assertEqual(assignment["action_end_wall_s"], 110.0)
            receipt["events"].append(dict(receipt["events"][0]))
            path.write_text(json.dumps(receipt))
            with self.assertRaisesRegex(ValueError, "exactly one"):
                read_attempt(root, path)
            receipt["events"] = receipt["events"][:1]
            path.write_text(json.dumps(receipt))
            with self.assertRaisesRegex(ValueError, "exactly one"):
                read_attempt(root, path)

    def test_delayed_delivery_of_old_map_is_not_fresh_and_future_stamp_fails(self):
        self.assertFalse(fresh_source(99, 100, 120))
        self.assertFalse(fresh_source(100, 100, 120))
        self.assertTrue(fresh_source(101, 100, 120))
        with self.assertRaisesRegex(ValueError, "future"):
            fresh_source(121, 100, 120)

    def test_newer_accepted_scoop_blocks_old_assignment_even_without_result(self):
        assignment = {"attempt_file": "scoop-10.json", "attempt_id": "scoop-10",
                      "goal_id": "4c72602b93b44857a268492244aacefc", "action_start_wall_s": 100.0}
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "session10"
            root.mkdir()
            path = root / "scoop-11.json"
            event = {"event": "goal_response", "accepted": False,
                     "goal_id": "4c72602b93b44857a268492244aacefd", "wall_s": 120.0}
            path.write_text(json.dumps({"events": [event]}))
            require_latest_attempt(root, assignment)
            event["accepted"] = True
            path.write_text(json.dumps({"events": [event]}))
            with self.assertRaisesRegex(ValueError, "Another scoop has started"):
                require_latest_attempt(root, assignment)

    def test_newer_scoop_in_sibling_run_blocks_old_assignment(self):
        assignment = {"attempt_file": "provenance/scoop-11.json", "attempt_id": "scoop-11",
                      "goal_id": "4c72602b93b44857a268492244aacefc", "action_start_wall_s": 100.0}
        with tempfile.TemporaryDirectory() as temporary:
            selected_run = Path(temporary) / "multiref_session10"
            (selected_run / "provenance").mkdir(parents=True)
            sibling = Path(temporary) / "sobol_session12" / "provenance"
            sibling.mkdir(parents=True)
            (sibling / "scoop09-measurement-gap.json").write_text('{"samples": 200}')
            path = sibling / "scoop-12.json"
            event = {"event": "goal_response", "accepted": True,
                     "goal_id": "4c72602b93b44857a268492244aacefd", "wall_s": 90.0}
            path.write_text(json.dumps({"events": [event]}))
            require_latest_attempt(selected_run, assignment)
            event["wall_s"] = 120.0
            path.write_text(json.dumps({"events": [event]}))
            with self.assertRaisesRegex(ValueError, "sobol_session12/provenance/scoop-12.json"):
                require_latest_attempt(selected_run, assignment)

    def test_full_map_round_trip_retains_source_stamp_and_never_overwrites(self):
        msg = GridMap()
        msg.header.stamp.sec = 102
        msg.header.frame_id = "map"
        msg.info.resolution = 0.5
        msg.info.length_x = 1.0
        msg.info.length_y = 1.0
        msg.info.pose.orientation.w = 1.0
        msg.layers = ["elevation", "desired_elevation", "dug_zone"]
        msg.data = [Float32MultiArray(data=values) for values in (
            [1.0, 2.0, float("nan"), 3.0], [-1.5] * 4, [0.0, 0.0, 1.0, 1.0])]
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "map"
            report = save_and_verify_map(path, msg, 100_000_000_000, 103_000_000_000)
            self.assertEqual(report["source_stamp_ns"], 102_000_000_000)
            self.assertEqual(report["layers"], list(msg.layers))
            self.assertEqual(report["finite_cells"]["elevation"], 3)
            self.assertEqual(report["received_wall_ns"], 103_000_000_000)
            self.assertEqual(len(report["cdr_sha256"]), 64)
            with self.assertRaises(FileExistsError):
                save_and_verify_map(path, msg, 100_000_000_000, 103_000_000_000)
            with self.assertRaisesRegex(ValueError, "newer"):
                save_and_verify_map(Path(temporary) / "stale", msg, 104_000_000_000, 105_000_000_000)


if __name__ == "__main__":
    unittest.main()
