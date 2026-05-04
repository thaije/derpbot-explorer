import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import time as real_time
import multiprocessing as mp
import queue as pyqueue
from threading import Thread

# Mock ROS2 and related modules before importing Detector
mock_rclpy = Mock()
mock_rclpy_node = Mock()
mock_rclpy_node.Node = Mock
mock_rclpy.node = mock_rclpy_node

mock_rclpy_qos = Mock()
mock_rclpy_qos.QoSProfile = Mock
mock_rclpy_qos.ReliabilityPolicy = Mock()
mock_rclpy_qos.DurabilityPolicy = Mock()
mock_rclpy.qos = mock_rclpy_qos

sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = mock_rclpy_node
sys.modules['rclpy.qos'] = mock_rclpy_qos

# Mock sensor_msgs
mock_sensor_msgs = Mock()
mock_sensor_msgs_msg = Mock()
mock_sensor_msgs_msg.Image = Mock()
sys.modules['sensor_msgs'] = mock_sensor_msgs
sys.modules['sensor_msgs.msg'] = mock_sensor_msgs_msg

# Now import Detector
from agent.detector import Detector, DetectionResult


class TestDetectorWatchdog(unittest.TestCase):
    def setUp(self):
        # Mock the ROS node
        self.mock_node = Mock()
        self.mock_logger = Mock()
        self.mock_node.get_logger.return_value = self.mock_logger

        # Patch multiprocessing to prevent real subprocesses
        self.mp_patcher = patch('agent.detector.mp', spec=mp)
        self.mock_mp = self.mp_patcher.start()

        # Mock spawn context
        self.mock_ctx = Mock()
        self.mock_mp.get_context.return_value = self.mock_ctx

        # Mock Queue
        self.mock_mp_queue = Mock()
        self.mock_ctx.Queue.return_value = self.mock_mp_queue

        # Mock Event
        self.mock_mp_event = Mock()
        self.mock_ctx.Event.return_value = self.mock_mp_event

        # Mock Process
        self.mock_worker = Mock()
        self.mock_worker.is_alive.return_value = True
        self.mock_ctx.Process.return_value = self.mock_worker

        # Patch threading.Thread to prevent real thread start
        self.thread_patcher = patch('agent.detector.threading.Thread')
        self.mock_thread = self.thread_patcher.start()
        self.mock_thread_instance = Mock()
        self.mock_thread.return_value = self.mock_thread_instance

        # Create Detector instance (no subscriber to avoid ROS callbacks)
        self.targets = ["fire_extinguisher"]
        self.detector = Detector(
            node=self.mock_node,
            targets=self.targets,
            create_subscriber=False
        )

        # Replace mocked queues with controllable mocks
        self.detector._mp_results = Mock()
        self.detector._mp_frames = Mock()
        self.detector._mp_frames.empty.return_value = False  # default

        # Use real queue.Queue for detections to check results
        self.detector.detections = pyqueue.Queue(maxsize=100)

        # Reset worker alive status
        self.mock_worker.is_alive.return_value = True

        # Track _restart_worker calls
        self.restart_call_count = 0
        self.frame_q_empty_logged = None
        def mock_restart():
            self.restart_call_count += 1
            # Log frame queue state as in the fix
            self.frame_q_empty_logged = self.detector._mp_frames.empty()
        self.detector._restart_worker = mock_restart

        # Mock time.monotonic to control time
        self.current_time = 0.0
        self.time_patcher = patch(
            'time.monotonic',
            side_effect=lambda: self.current_time
        )
        self.time_patcher.start()

        # Set running to True by default
        self.detector._running = True

    def tearDown(self):
        self.mp_patcher.stop()
        self.thread_patcher.stop()
        self.time_patcher.stop()

    def test_watchdog_no_fire_on_warnings(self):
        """Test that repeated warning messages reset watchdog, no restart."""
        # Simulate 200 warning results (enough for >90s of 0.5s ticks)
        warning_result = {"warning": "inference error"}
        self.detector._mp_results.get.side_effect = [warning_result] * 200

        # Run relay loop in a thread
        relay_thread = Thread(target=self.detector._relay_loop)
        relay_thread.start()

        # Let it process some results
        real_time.sleep(0.1)

        # Stop the loop
        self.detector._running = False
        relay_thread.join(timeout=1)

        # Assert no restart was called
        self.assertEqual(
            self.restart_call_count, 0,
            "Watchdog should not fire on repeated warning messages"
        )

    def test_relay_thread_survives_malformed_result(self):
        """Test that malformed result doesn't kill relay thread."""
        # Malformed result: no expected keys
        malformed_result = {"bad_key": "oops"}
        # Valid detection result
        valid_detection = {
            "detections": [
                {
                    "class_name": "fire_extinguisher",
                    "confidence": 0.5,
                    "cx_px": 100.0,
                    "cy_px": 100.0,
                    "w_px": 50.0,
                    "h_px": 50.0,
                    "stamp_sec": 0,
                    "stamp_nanosec": 0,
                }
            ]
        }

        # Sequence: malformed → valid → Empty (to exit)
        self.detector._mp_results.get.side_effect = [
            malformed_result, valid_detection, pyqueue.Empty
        ]

        # Run relay loop
        relay_thread = Thread(target=self.detector._relay_loop)
        relay_thread.start()

        real_time.sleep(0.1)

        self.detector._running = False
        relay_thread.join(timeout=1)

        # Check valid detection was processed
        try:
            det = self.detector.detections.get_nowait()
            self.assertIsInstance(det, DetectionResult)
            self.assertEqual(det.class_name, "fire_extinguisher")
        except pyqueue.Empty:
            self.fail("Valid detection not processed after malformed result")

    def test_relay_thread_survives_processing_exception(self):
        """Test that exception during detection processing doesn't kill thread."""
        # Bad detection: missing required key "class_name"
        bad_detection = {
            "detections": [
                {
                    # Missing "class_name"
                    "confidence": 0.5,
                    "cx_px": 100.0,
                    "cy_px": 100.0,
                    "w_px": 50.0,
                    "h_px": 50.0,
                    "stamp_sec": 0,
                    "stamp_nanosec": 0,
                }
            ]
        }
        # Valid detection after bad one
        valid_detection = {
            "detections": [
                {
                    "class_name": "fire_extinguisher",
                    "confidence": 0.5,
                    "cx_px": 100.0,
                    "cy_px": 100.0,
                    "w_px": 50.0,
                    "h_px": 50.0,
                    "stamp_sec": 0,
                    "stamp_nanosec": 0,
                }
            ]
        }

        self.detector._mp_results.get.side_effect = [
            bad_detection, valid_detection, pyqueue.Empty
        ]

        relay_thread = Thread(target=self.detector._relay_loop)
        relay_thread.start()

        real_time.sleep(0.1)

        self.detector._running = False
        relay_thread.join(timeout=1)

        # Check valid detection was processed
        try:
            det = self.detector.detections.get_nowait()
            self.assertEqual(det.class_name, "fire_extinguisher")
        except pyqueue.Empty:
            self.fail("Valid detection not processed after exception in bad detection")

        # Check error was logged
        self.mock_logger.error.assert_called()


if __name__ == "__main__":
    unittest.main()
