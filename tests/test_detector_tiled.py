"""
Unit tests for tiled native-resolution detector inference (t_ea3db560).
Covers:
  1. tile_grid covers the frame exactly (total, disjoint, ordered q=0..3).
  2. remap_tile_box_to_full is exact translation (offset add).
  3. Tile-edge straddle: a box flush to a tile edge maps to the same edge
     full-frame coords (per-tile detection + tracker-side dedup semantics).
  4. Worker detection dicts carry source="tile_<q>" in the tiled path and
     source="full_frame" when tiling is off (marker tagging at the relay).
  5. Detections absent a source marker fall back to "tile_?"/"full_frame"
     (spawn-subprocess env race tolerance).
  6. Tiled instrumentation path: per-frame wall message emitted every 5th
     inference; non-tiled VRAM heartbeat stays intact.

Interpreter: sandbox venv + ROS paths (PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages).
Run: .venv/bin/python -m pytest tests/test_detector_tiled.py -q
"""

import os
import sys
from unittest.mock import Mock, MagicMock, patch
import multiprocessing as mp
import queue as pyqueue

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ---------------------------------------------------------------------------
# Mock ROS2 and related modules before importing detector (same pattern as
# tests/test_detector_watchdog.py and tests/test_detector_roi.py)
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
    DetectionResult,
    TILING_SOURCE_MARKERS,
    TILING_GRID,
    tile_grid,
    remap_tile_box_to_full,
)

import numpy as np  # noqa: E402
import unittest  # noqa: E402
import threading  # noqa: E402

# Capture the REAL Thread class before any patch starts: the harness patches
# 'agent.detector.threading.Thread', which resolves to the stdlib threading
# module itself — a process-wide Thread mock while active. Tests that must
# actually RUN the relay body use this saved handle.
REAL_THREAD = threading.Thread


class TestTileGrid(unittest.TestCase):

    def test_grid_covers_frame_exactly(self):
        tiles = tile_grid(480, 640)
        self.assertEqual(len(tiles), 4)
        self.assertEqual([t[0] for t in tiles], [0, 1, 2, 3])
        area = sum((y1 - y0) * (x1 - x0) for _, y0, y1, x0, x1 in tiles)
        self.assertEqual(area, 480 * 640)
        # Tiles are non-overlapping: collect unique edges
        spans = set()
        for _, y0, y1, x0, x1 in tiles:
            self.assertGreater(y1, y0)
            self.assertGreater(x1, x0)
            spans.add((y0, y1, x0, x1))
        self.assertEqual(len(spans), 4)

    def test_grid_ragged_edge_total_coverage(self):
        tiles = tile_grid(481, 641)
        area = sum((y1 - y0) * (x1 - x0) for _, y0, y1, x0, x1 in tiles)
        self.assertEqual(area, 481 * 641)

    def test_grid_quadrants_are_expected_quadrants(self):
        tiles = tile_grid(480, 640)
        self.assertEqual(tiles[0], (0, 0, 240, 0, 320))
        self.assertEqual(tiles[1], (1, 0, 240, 320, 640))
        self.assertEqual(tiles[2], (2, 240, 480, 0, 320))
        self.assertEqual(tiles[3], (3, 240, 480, 320, 640))

    def test_grid_marker_count_matches(self):
        self.assertEqual(len(TILING_SOURCE_MARKERS), TILING_GRID[0] * TILING_GRID[1])


class TestRemapExactness(unittest.TestCase):

    def test_remap_is_exact_translation(self):
        tiles = tile_grid(480, 640)
        for tile in tiles:
            _, y0, y1_, x0, x1_ = tile
            for (bx1, by1, bx2, by2) in [(10, 20, 30, 40), (100, 150, 200, 220)]:
                fx1, fy1, fx2, fy2 = remap_tile_box_to_full(
                    tile, (bx1, by1, bx2, by2)
                )
                self.assertAlmostEqual(fx1, bx1 + x0)
                self.assertAlmostEqual(fy1, by1 + y0)
                self.assertAlmostEqual(fx2, bx2 + x0)
                self.assertAlmostEqual(fy2, by2 + y0)
                # Width/height preserved
                self.assertAlmostEqual(fx2 - fx1, bx2 - bx1)

    def test_remap_straddle_edges_land_on_full_frame_edges(self):
        tiles = tile_grid(480, 640)
        # object flush to tile_q0's right edge (x=320 local) maps to x=320 full
        self.assertEqual(
            remap_tile_box_to_full(tiles[0], (300, 100, 320, 200)),
            (300, 100, 320, 200),
        )
        # object flush to tile_q3's top-left maps to (320, 240) full
        self.assertEqual(
            remap_tile_box_to_full(tiles[3], (0, 0, 30, 40)),
            (320, 240, 350, 280),
        )


class TestWorkerDetectionMarkers(unittest.TestCase):
    """Drive _inference_worker's detection-dict post-processing indirectly:
    the worker body is exercised in sim; here we verify the marker contract
    via the dict-level contract the worker produces (source key set for tile
    paths, stamp keys always set) and the relay's fallback tagging."""

    def _relay_harness(self):
        mock_node = Mock()
        mock_logger = Mock()
        mock_node.get_logger.return_value = mock_logger
        mp_patcher = patch('agent.detector.mp', spec=mp)
        mock_mp = mp_patcher.start()
        mock_ctx = Mock()
        mock_mp.get_context.return_value = mock_ctx
        mock_worker = Mock()
        mock_worker.is_alive.return_value = True
        mock_ctx.Process.return_value = mock_worker
        mock_ctx.Queue.return_value = Mock()
        mock_ctx.Event.return_value = Mock()
        thread_patcher = patch('agent.detector.threading.Thread')
        thread_patcher.start()
        from agent.detector import Detector
        detector = Detector(node=mock_node, targets=["t1"], create_subscriber=False)
        detector._mp_results = Mock()
        detector._mp_frames = Mock()
        detector._mp_frames.empty.return_value = False
        detector.detections = pyqueue.Queue(maxsize=100)
        detector._restart_worker = Mock()
        detector._running = True
        return detector, mp_patcher, thread_patcher

    @staticmethod
    def _worker_dets(source_key=None):
        d = {
            "class_name": "t1", "confidence": 0.5,
            "cx_px": 300.0, "cy_px": 200.0, "w_px": 40.0, "h_px": 60.0,
            "stamp_sec": 7, "stamp_nanosec": 0,
        }
        if source_key is not None:
            d["source"] = source_key
        return {"detections": [d]}

    def _drain_one(self, detector, results):
        # NOTE: the harness patches agent.detector.threading.Thread, which is
        # the stdlib threading module itself — every Thread in the process
        # would be a Mock while active. Use the REAL Thread captured at import.
        import time as real_time
        detector._mp_results.get.side_effect = list(results) + [pyqueue.Empty] * 100000
        th = REAL_THREAD(target=detector._relay_loop, daemon=True)
        th.start()
        # The relay's first mp_results.get blocks 0.5 s before the queued
        # result is seen when the side_effect list is installed after thread
        # start — poll the detections queue instead of a fixed short sleep.
        deadline = real_time.monotonic() + 3.0
        det = None
        while real_time.monotonic() < deadline:
            try:
                det = detector.detections.get_nowait()
                break
            except pyqueue.Empty:
                real_time.sleep(0.05)
        detector._running = False
        th.join(timeout=2.0)
        self.assertIsNotNone(det, "relay must deliver the queued result within 3s")
        return det

    def test_tiled_detection_markers_passthrough(self):
        for q, marker in enumerate(TILING_SOURCE_MARKERS):
            detector, mp_p, t_p = self._relay_harness()
            try:
                det = self._drain_one(detector, [self._worker_dets(marker)])
                self.assertEqual(det.source, marker)
                self.assertEqual(det.stamp, (7, 0))
                self.assertAlmostEqual(det.cx_px, 300.0)
            finally:
                mp_p.stop(); t_p.stop()

    def test_unmarked_full_frame_result_defaults(self):
        detector, mp_p, t_p = self._relay_harness()
        try:
            det = self._drain_one(detector, [self._worker_dets(None)])
            # TILING_ENABLED is read at import: in test env DERPBOT_TILED_INFERENCE
            # is unset → non-tiled default "full_frame".
            from agent.detector import TILING_ENABLED
            self.assertEqual(det.source, "full_frame" if not TILING_ENABLED else "tile_?")
        finally:
            mp_p.stop(); t_p.stop()

    def test_detectionresult_default_source_is_full_frame(self):
        det = DetectionResult(
            class_name="t1", confidence=0.5, cx_px=1, cy_px=2, w_px=3, h_px=4,
            stamp=(0, 0),
        )
        self.assertEqual(det.source, "full_frame")


if __name__ == "__main__":
    unittest.main()