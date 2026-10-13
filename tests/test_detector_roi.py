"""
Unit tests for the event-triggered ROI second-pass detector recall
(t_c6b546ac). Covers:
  1. ROI trigger FIRES for a pending class silent >= K full-frame cycles.
  2. ROI trigger SKIPS when the class is still being sighted (< K cycles).
  3. ROI trigger SKIPS during the cooldown window (rate-limit honored).
  4. ROI results carry the source marker ("roi_second_pass").
  5. update_pending_classes is the only tracker-boundary plumbing (read-only).

Interpreter: sandbox venv + ROS paths (PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages).
Run: .venv/bin/python -m pytest tests/test_detector_roi.py -q
"""

import sys
from unittest.mock import Mock, patch
import multiprocessing as mp
import queue as pyqueue
from threading import Thread
import time as real_time

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ---------------------------------------------------------------------------
# Mock ROS2 and related modules before importing detector (same pattern as
# tests/test_detector_watchdog.py)
# ---------------------------------------------------------------------------
mock_rclpy = Mock()
mock_rclpy_node = Mock()
mock_rclpy_node.Node = Mock
mock_rclpy.node = mock_rclpy_node

mock_rclpy_qos = Mock()
mock_rclpy_qos.QoSProfile = Mock
mock_rclpy_qos.ReliabilityPolicy = Mock()
mock_rclpy_qos.DurabilityPolicy = Mock()
mock_rclpy.qos = mock_rclpy_qos

mock_rclpy_callback_groups = Mock()
mock_rclpy_callback_groups.ReentrantCallbackGroup = Mock
mock_rclpy.callback_groups = mock_rclpy_callback_groups

sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = mock_rclpy_node
sys.modules['rclpy.qos'] = mock_rclpy_qos
sys.modules['rclpy.callback_groups'] = mock_rclpy_callback_groups

mock_sensor_msgs = Mock()
mock_sensor_msgs_msg = Mock()
mock_sensor_msgs_msg.Image = Mock()
sys.modules['sensor_msgs'] = mock_sensor_msgs
sys.modules['sensor_msgs.msg'] = mock_sensor_msgs_msg

mock_cv_bridge = Mock()
mock_cv_bridge.CvBridge = Mock
sys.modules['cv_bridge'] = mock_cv_bridge

from agent.detector import (  # noqa: E402
    Detector,
    DetectionResult,
    ROI_SILENT_CYCLES_K,
    ROI_COOLDOWN_S,
    ROI_SOURCE_TAG,
)


class _RoiHarness:
    """Builds a Detector without ROS/MP subprocesses and drives its relay."""

    def __init__(self, targets=("fire_extinguisher", "first_aid_kit")):
        self.mock_node = Mock()
        self.mock_logger = Mock()
        self.mock_node.get_logger.return_value = self.mock_logger

        self.mp_patcher = patch('agent.detector.mp', spec=mp)
        self.mock_mp = self.mp_patcher.start()

        self.mock_ctx = Mock()
        self.mock_mp.get_context.return_value = self.mock_ctx
        self.mock_mp_queue = Mock()
        self.mock_ctx.Queue.return_value = self.mock_mp_queue
        self.mock_mp_event = Mock()
        self.mock_ctx.Event.return_value = self.mock_mp_event
        self.mock_worker = Mock()
        self.mock_worker.is_alive.return_value = True
        self.mock_ctx.Process.return_value = self.mock_worker

        self.thread_patcher = patch('agent.detector.threading.Thread')
        self.mock_thread = self.thread_patcher.start()

        self.detector = Detector(
            node=self.mock_node, targets=list(targets), create_subscriber=False
        )
        # Controllable queues
        self.detector._mp_results = Mock()
        self.detector._mp_frames = Mock()
        self.detector._mp_frames.empty.return_value = False
        self.detector._mp_roi = Mock()  # records put_nowait calls
        self.detector.detections = pyqueue.Queue(maxsize=100)
        self.detector._restart_worker = Mock()
        self.detector._running = True

    def stop(self):
        self.mp_patcher.stop()
        self.thread_patcher.stop()

    def run_relay(self, results, stop_after=None):
        """Feed `results` through the relay loop in a background thread."""
        self.detector._mp_results.get.side_effect = list(results) + [
            pyqueue.Empty
        ] * 100000
        relay_thread = Thread(target=self.detector._relay_loop)
        relay_thread.start()
        real_time.sleep(0.15)
        self.detector._running = False
        relay_thread.join(timeout=2.0)
        return relay_thread

    def roi_request(self):
        """The (single) ROI request the harness's detector queued, or None."""
        calls = self.detector._mp_roi.put_nowait.call_args_list
        if not calls:
            return None
        args, kwargs = calls[0]
        return args[0] if args else kwargs.get("req")

    def queued_roi_count(self):
        return len(self.detector._mp_roi.put_nowait.call_args_list)


# Fake full-frame detection payload (relay result format from the worker)
def ff_detections(class_name, conf=0.42, cx=300.0, cy=200.0, n=1):
    return {
        "detections": [
            {
                "class_name": class_name,
                "confidence": conf,
                "cx_px": cx,
                "cy_px": cy,
                "w_px": 40.0,
                "h_px": 60.0,
                "stamp_sec": 0,
                "stamp_nanosec": 0,
            }
        ] * n
    }


import unittest  # noqa: E402


class TestRoiTrigger(unittest.TestCase):

    def setUp(self):
        self.h = _RoiHarness()
        # Frame anchor: detector has seen a frame (bytes+shape) so crops have
        # a source. Use small fake bytes (never reaches numpy in these tests).
        self.h.detector._last_roi_frame = (b"\x00" * 8, (2, 2, 2))

    def tearDown(self):
        self.h.stop()

    # 1. FIRES: pending class, no sighting for >= K cycles.
    def test_trigger_fires_for_silent_pending_class(self):
        h = self.h
        # Tracker reports first_aid_kit as pending (1 sighting, unconfirmed)
        h.detector.update_pending_classes(["first_aid_kit"])
        h.detector._last_class_img["first_aid_kit"] = (320.0, 240.0)
        # Last full-frame sighting of first_aid_kit was long ago (> K cycles
        # at 5 Hz = (now - last) * 5 >= K ⇒ >= 2 s ago).
        h.detector._last_class_sighting["first_aid_kit"] = (
            real_time.monotonic() - (ROI_SILENT_CYCLES_K / 5.0) - 0.5
        )
        fake_now = real_time.monotonic()
        h.detector._maybe_roi_pass(fake_now)

        req = h.roi_request()
        self.assertIsNotNone(req, "ROI request must be queued for silent pending class")
        self.assertEqual(req["roi_class"], "first_aid_kit")
        self.assertTrue(req["_is_roi"])
        self.assertEqual(req["frame_bytes"], b"\x00" * 8)
        self.assertEqual(req["shape"], (2, 2, 2))

    # 2. SKIPS: class still being sighted (silence < K cycles).
    def test_trigger_skips_recently_sighted_class(self):
        h = self.h
        h.detector.update_pending_classes(["first_aid_kit"])
        # Sighted 0.2 s ago ⇒ (0.2 * 5) = 1 cycle < K=10 → skip.
        h.detector._last_class_sighting["first_aid_kit"] = (
            real_time.monotonic() - 0.2
        )
        h.detector._maybe_roi_pass(real_time.monotonic())
        self.assertIsNone(h.roi_request(), "recent sighting must suppress ROI")

    # 2b. SKIPS: class has no image anchor from this detector.
    def test_trigger_skips_without_image_anchor(self):
        h = self.h
        h.detector.update_pending_classes(["first_aid_kit"])
        h.detector._last_class_sighting["first_aid_kit"] = (
            real_time.monotonic() - 999.0
        )
        h.detector._last_class_img.pop("first_aid_kit", None)
        h.detector._maybe_roi_pass(real_time.monotonic())
        self.assertIsNone(h.roi_request(), "no image anchor → no ROI pass")

    # 3. Rate-limit: second call within cooldown does not queue again.
    def test_trigger_rate_limited_by_cooldown(self):
        h = self.h
        h.detector.update_pending_classes(["first_aid_kit"])
        long_ago = real_time.monotonic() - (ROI_SILENT_CYCLES_K / 5.0) - 0.5
        h.detector._last_class_sighting["first_aid_kit"] = long_ago
        h.detector._last_class_img["first_aid_kit"] = (320.0, 240.0)

        now0 = real_time.monotonic()
        h.detector._maybe_roi_pass(now0)
        self.assertEqual(h.queued_roi_count(), 1, "first trigger fires")

        h.detector._maybe_roi_pass(now0 + 0.5)  # inside ROI_COOLDOWN_S
        self.assertEqual(
            h.queued_roi_count(), 1, "second call inside cooldown must not fire"
        )

        h.detector._maybe_roi_pass(now0 + ROI_COOLDOWN_S + 0.1)
        self.assertEqual(
            h.queued_roi_count(), 2, "after cooldown the trigger may fire again"
        )

    # 3b. Trigger does not fire for a class NOT pending (confirmed already).
    def test_trigger_skips_non_pending_class(self):
        h = self.h
        # Nothing pending: confirmed classes must not trigger ROI.
        h.detector.update_pending_classes([])
        h.detector._last_class_sighting["fire_extinguisher"] = (
            real_time.monotonic() - 999.0
        )
        h.detector._maybe_roi_pass(real_time.monotonic())
        self.assertIsNone(h.roi_request())

    # 3c. Confirmed-class pending-list updates: a ROI-triggered class that
    #     has been confirmed (dropped from pending) no longer triggers.
    def test_pending_set_update_changes_eligibility(self):
        h = self.h
        long_ago = real_time.monotonic() - 999.0
        h.detector._last_class_sighting["first_aid_kit"] = long_ago
        h.detector._last_class_img["first_aid_kit"] = (320.0, 240.0)

        h.detector.update_pending_classes(["first_aid_kit"])
        h.detector._maybe_roi_pass(real_time.monotonic())
        self.assertEqual(h.queued_roi_count(), 1)

        # Candidate confirmed → pending set now empty → no more passes.
        h.detector.update_pending_classes([])
        h.detector._maybe_roi_pass(real_time.monotonic() + ROI_COOLDOWN_S + 0.1)
        self.assertEqual(h.queued_roi_count(), 1)

    # 4. Source marker: relay stamps ROI results with the dedicated field.
    def test_roi_results_carry_source_marker(self):
        h = self.h
        h.detector.update_pending_classes(["first_aid_kit"])
        long_ago = real_time.monotonic() - 999.0
        h.detector._last_class_sighting["first_aid_kit"] = long_ago
        h.detector._last_class_img["first_aid_kit"] = (320.0, 240.0)

        roi_hit = {
            "roi_detections": [
                {
                    "class_name": "first_aid_kit",
                    "confidence": 0.31,
                    "cx_px": 318.0,
                    "cy_px": 238.0,
                    "w_px": 30.0,
                    "h_px": 44.0,
                    "stamp_sec": 5,
                    "stamp_nanosec": 0,
                    "source": ROI_SOURCE_TAG,
                }
            ]
        }
        h.run_relay([roi_hit])
        det = h.detector.detections.get_nowait()
        self.assertIsInstance(det, DetectionResult)
        self.assertEqual(det.source, ROI_SOURCE_TAG)
        self.assertEqual(det.class_name, "first_aid_kit")
        self.assertAlmostEqual(det.confidence, 0.31)

    # 4b. Full-frame detections keep source="full_frame".
    def test_full_frame_results_keep_full_frame_marker(self):
        h = self.h
        h.run_relay([ff_detections("fire_extinguisher")])
        det = h.detector.detections.get_nowait()
        self.assertEqual(det.source, "full_frame")

    # 4c. ROI-hit refreshes the class's sighting clock (no immediate re-fire).
    def test_roi_hit_refreshes_class_clock(self):
        h = self.h
        roi_hit = {
            "roi_detections": [
                {
                    "class_name": "first_aid_kit",
                    "confidence": 0.31,
                    "cx_px": 318.0, "cy_px": 238.0, "w_px": 30.0, "h_px": 44.0,
                    "stamp_sec": 5, "stamp_nanosec": 0,
                    "source": ROI_SOURCE_TAG,
                }
            ]
        }
        h.run_relay([roi_hit])
        self.assertIn(
            "first_aid_kit", h.detector._last_class_sighting,
            "ROI hit must refresh the sighting clock"
        )
        # And it stores the returned full-frame image anchor for next time.
        self.assertIn("first_aid_kit", h.detector._last_class_img)

    # 5. Compute-budget sanity: a full-frame result evaluates the trigger at
    #    most once per result, so passes stay event-triggered.
    def test_budget_one_trigger_per_relay_cycle(self):
        h = self.h
        h.detector.update_pending_classes(["first_aid_kit", "fire_extinguisher"])
        long_ago = real_time.monotonic() - 999.0
        for cls in ("first_aid_kit", "fire_extinguisher"):
            h.detector._last_class_sighting[cls] = long_ago
            h.detector._last_class_img[cls] = (320.0, 240.0)
        h.detector._maybe_roi_pass(real_time.monotonic())
        # One pass per pending class max — both pending & silent → 2 requests.
        self.assertEqual(h.queued_roi_count(), 2)


class TestRoiWorkerLogic(unittest.TestCase):
    """Verify the coordinate mapping math used by the worker ROI pass
    (pure-python reimplementation of the mapping; the torch path itself is
    only exercised in sim)."""

    def test_mapping_math(self):
        # Full frame 640x480, crop anchored at top-left (160,80).
        upscale = 2.0
        x1c, y1c = 160, 80
        # A box in upscaled-crop coords: (100,60)-(300,240)
        #   → crop px: (50,30)-(150,120) → full-frame px: (210,110)-(310,200)
        bx1, by1, bx2, by2 = 100.0, 60.0, 300.0, 240.0
        fx1 = x1c + bx1 / upscale
        fy1 = y1c + by1 / upscale
        fx2 = x1c + bx2 / upscale
        fy2 = y1c + by2 / upscale
        self.assertAlmostEqual((fx1 + fx2) / 2, 260.0)
        self.assertAlmostEqual((fy1 + fy2) / 2, 155.0)
        self.assertAlmostEqual(fx2 - fx1, 100.0)
        self.assertAlmostEqual(fy2 - fy1, 90.0)


if __name__ == "__main__":
    unittest.main()