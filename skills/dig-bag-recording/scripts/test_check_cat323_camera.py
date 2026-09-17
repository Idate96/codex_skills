"""Check live MCAP tails and keyframe-before-goal admission without camera traffic."""
import tempfile
from pathlib import Path
import unittest

from foxglove_msgs.msg import CompressedVideo
from mcap.writer import CompressionType, Writer
from rclpy.serialization import serialize_message

from check_cat323_camera import inspect_bag


class CameraAdmissionTests(unittest.TestCase):
    def test_keyframe_must_precede_goal_and_survives_unfinished_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "camera_0.mcap"
            with path.open("wb") as stream:
                writer = Writer(stream, compression=CompressionType.NONE)
                writer.start()
                schema = writer.register_schema("foxglove_msgs/msg/CompressedVideo", "ros2msg", b"")
                channel = writer.register_channel("/hal/perception/main/compressed_video", "cdr", schema)
                for stamp, types in [(100, [1]), (200, [32, 33, 34, 19])]:
                    msg = CompressedVideo()
                    msg.timestamp.nanosec = stamp
                    msg.format = "h265"
                    msg.data = b"".join(b"\x00\x00\x01" + bytes([kind << 1, 1, 128]) for kind in types)
                    writer.add_message(channel, stamp, serialize_message(msg), stamp)
                writer.finish()
            self.assertFalse(inspect_bag(root, 150)["passed"])
            self.assertTrue(inspect_bag(root, 250)["passed"])
            original = path.read_bytes()
            path.write_bytes(original[:-4])
            report = inspect_bag(root, 250)
            self.assertTrue(report["passed"])
            self.assertFalse(report["all_files_complete"])
            path.write_bytes(original[:12])
            report = inspect_bag(root, 250)
            self.assertFalse(report["passed"])
            self.assertFalse(report["all_files_complete"])


if __name__ == "__main__":
    unittest.main()
