"""
DerpBot Autonomous Agent — Approach 1: Classical Modular Pipeline
  slam_toolbox  → /map
  Nav2          → NavigateToPose
  FrontierExplorer → systematic coverage
  Detector + DepthProjector + Tracker → /derpbot_0/detections

State machine: INIT → EXPLORE → DONE

Run (once Nav2 + slam_toolbox are up):
  python3 agent_node.py
Or via launch file:
  ros2 launch derpbot_autonomy.launch.py
"""

import argparse
import logging
import sys
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Wall-clock baseline for startup timing (set once at import time)
_WALL_T0 = time.time()

from mission_client import fetch_mission
from frontier_explorer import FrontierExplorer
from detector import Detector
from depth_projector import DepthProjector
from tracker import Tracker

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("agent_node")


class AgentNode(Node):
    def __init__(self, no_perception: bool = False, no_subscribers: bool = False):
        super().__init__(
            "derpbot_agent",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True
                )
            ],
        )
        self._state = "INIT"
        self._done_event = threading.Event()
        self._no_perception = no_perception
        self._no_subscribers = no_subscribers
        self._t_start = self.get_clock().now().nanoseconds / 1e9
        self._logger = self.get_logger()

        self._logger.info("[STARTUP TIMING] AgentNode init begin")

        # --- Fetch mission (blocks until server is ready) ---
        t0 = self.get_clock().now().nanoseconds / 1e9
        try:
            mission = fetch_mission()
        except RuntimeError as exc:
            self._logger.error(f"AgentNode: {exc}")
            sys.exit(1)
        t1 = self.get_clock().now().nanoseconds / 1e9
        self._logger.info(
            f"[STARTUP TIMING] fetch_mission: +{t1 - t0:.2f}s (total {t1 - self._t_start:.2f}s)"
        )

        self._logger.info(
            f"AgentNode: mission received — targets: {mission.targets}, time_limit: {mission.time_limit}s"
        )
        self._time_limit = mission.time_limit

        if not mission.targets:
            self._logger.warning(
                "AgentNode: no targets in mission — will explore only."
            )

        # --- Wire up perception pipeline (skipped when --no-perception) ---
        if no_perception:
            self._logger.info("AgentNode: perception disabled (--no-perception).")
            self._detector = None
            self._depth_projector = None
            self._tracker = None
            t2 = self.get_clock().now().nanoseconds / 1e9
            self._logger.info(
                f"[STARTUP TIMING] no-perception skip: +{t2 - t1:.2f}s (total {t2 - self._t_start:.2f}s)"
            )
        else:
            t2_start = self.get_clock().now().nanoseconds / 1e9
            self._detector = Detector(self, targets=mission.targets, create_subscriber=not no_subscribers)
            self._detector.start()
            t2_end = self.get_clock().now().nanoseconds / 1e9
            self._logger.info(
                f"[STARTUP TIMING] detector spawn+start: +{t2_end - t2_start:.2f}s (total {t2_end - self._t_start:.2f}s)"
            )

            self._depth_projector = DepthProjector(self, create_subscriber=not no_subscribers)

            self._tracker = Tracker(
                node=self,
                detector=self._detector,
                depth_projector=self._depth_projector,
                targets=mission.targets,
            )
            t3 = self.get_clock().now().nanoseconds / 1e9
            self._logger.info(
                f"[STARTUP TIMING] tracker init: +{t3 - t2_end:.2f}s (total {t3 - self._t_start:.2f}s)"
            )

        # --- Frontier explorer ---
        t4_start = self.get_clock().now().nanoseconds / 1e9
        self._explorer = FrontierExplorer(
            node=self,
            done_callback=self._on_exploration_done,
        )
        t4_end = self.get_clock().now().nanoseconds / 1e9
        self._logger.info(
            f"[STARTUP TIMING] frontier explorer init: +{t4_end - t4_start:.2f}s (total {t4_end - self._t_start:.2f}s)"
        )

        self._state = "EXPLORE"
        self._logger.info("AgentNode: EXPLORE — starting frontier exploration.")

        # Capture wall-clock time just before calling explorer.start()
        t_start_wall = time.time()
        self._explorer.start()
        t_end_wall = time.time()
        t5 = self.get_clock().now().nanoseconds / 1e9
        self._logger.info(
            f"[STARTUP TIMING] explorer.start() call: +{t_end_wall - t_start_wall:.2f}s wall (sim {t5 - self._t_start:.2f}s)"
        )
        self._logger.info(
            f"[STARTUP TIMING] explorer.start(): +{t5 - t4_end:.2f}s (total {t5 - self._t_start:.2f}s)"
        )

        # Enforce time limit — fire done_event when the clock runs out
        self._time_limit_timer = threading.Timer(self._time_limit, self._on_time_limit)
        self._time_limit_timer.daemon = True
        self._time_limit_timer.start()
        self._logger.info(
            f"[STARTUP TIMING] time limit timer started ({self._time_limit}s)"
        )

    # ------------------------------------------------------------------
    # Done callback (called from FrontierExplorer when no frontiers remain)
    # ------------------------------------------------------------------

    def _on_exploration_done(self) -> None:
        self.get_logger().info(
            "AgentNode: exploration complete — transitioning to DONE."
        )
        self._state = "DONE"
        self._time_limit_timer.cancel()
        self._done_event.set()

    def _on_time_limit(self) -> None:
        self.get_logger().info("AgentNode: time limit reached — stopping exploration.")
        self._state = "DONE"
        self._explorer.stop()
        self._done_event.set()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        self.get_logger().info("AgentNode: shutting down.")
        self._explorer.stop()
        if self._tracker is not None:
            self._tracker.stop()


def main():
    parser = argparse.ArgumentParser(description="DerpBot agent")
    parser.add_argument(
        "--no-perception",
        action="store_true",
        default=False,
        help="Disable OWLv2 detector/tracker; run nav+exploration only.",
    )
    parser.add_argument(
        "--no-subscribers",
        action="store_true",
        default=False,
        help="Disable image/depth subscribers (GIL probe); detector subprocess still runs.",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = AgentNode(no_perception=args.no_perception, no_subscribers=args.no_subscribers)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # Spin in background thread while we wait for exploration to finish
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node._done_event.wait()  # blocks until exploration complete
        node.get_logger().info("AgentNode: mission DONE.")
    except KeyboardInterrupt:
        node.get_logger().info("AgentNode: interrupted.")
    finally:
        node.shutdown()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
