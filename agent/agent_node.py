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

import logging
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

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
    def __init__(self):
        super().__init__("derpbot_agent", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)
        ])
        self._state = "INIT"
        self._done_event = threading.Event()

        self.get_logger().info("AgentNode: INIT — fetching mission…")

        # --- Fetch mission (blocks until server is ready) ---
        try:
            mission = fetch_mission()
        except RuntimeError as exc:
            self.get_logger().error(f"AgentNode: {exc}")
            sys.exit(1)

        self.get_logger().info(
            f"AgentNode: mission received — targets: {mission.targets}, time_limit: {mission.time_limit}s"
        )
        self._time_limit = mission.time_limit

        if not mission.targets:
            self.get_logger().warning("AgentNode: no targets in mission — will explore only.")

        # --- Wire up perception pipeline ---
        self._detector = Detector(self, targets=mission.targets)
        self._detector.start()

        self._depth_projector = DepthProjector(self)

        self._tracker = Tracker(
            node=self,
            detector=self._detector,
            depth_projector=self._depth_projector,
            targets=mission.targets,
        )

        # --- Frontier explorer ---
        self._explorer = FrontierExplorer(
            node=self,
            done_callback=self._on_exploration_done,
        )

        self._state = "EXPLORE"
        self.get_logger().info("AgentNode: EXPLORE — starting frontier exploration.")
        self._explorer.start()

        # Enforce time limit — fire done_event when the clock runs out
        self._time_limit_timer = threading.Timer(self._time_limit, self._on_time_limit)
        self._time_limit_timer.daemon = True
        self._time_limit_timer.start()
        self.get_logger().info(f"AgentNode: time limit timer started ({self._time_limit}s).")

    # ------------------------------------------------------------------
    # Done callback (called from FrontierExplorer when no frontiers remain)
    # ------------------------------------------------------------------

    def _on_exploration_done(self) -> None:
        self.get_logger().info("AgentNode: exploration complete — transitioning to DONE.")
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
        self._tracker.stop()


def main():
    rclpy.init()
    node = AgentNode()

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
