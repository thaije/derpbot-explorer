# Nav2 Deep Research Prompt

## Context

We are running a Nav2 Jazzy autonomous exploration robot (DerpBot) in Gazebo simulation. The robot uses:
- SmacPlanner2D (global planner) + MPPI controller (local)
- slam_toolbox (async online SLAM, 5cm resolution)
- collision_monitor → cmd_vel pipeline
- inflation_radius: 0.25m, robot_radius: 0.22m
- Single GPU (RTX 2070 SUPER, shared between Gazebo renderer and OWLv2 inference)
- ROS 2 Jazzy, FastDDS with discovery server, speed=2 (RTF≈2)

We are trying to achieve robust, **generic** autonomous navigation — not brittle patches for specific scenarios. The robot must handle any environment, any obstacle configuration, including objects falling on or near the robot mid-run.

---

## Problems encountered

### 1. Robot ends up in inflation zone → "Start occupied" → all planning fails

After navigating to a frontier cell (free in the SLAM map), the robot arrives and SLAM updates with newly observed walls. Inflation from those walls reaches the robot's current cell, marking it LETHAL. SmacPlanner2D then refuses all subsequent goals with "Start occupied" or "no valid path found". The robot is permanently stuck unless something physically moves it.

**What we tried (didn't work robustly):**
- Calling `clear_global_costmap` once after failure — the LiDAR scan immediately re-inflates the same cell
- Calling Nav2 `backup` (0.3m) + `clear_global_costmap` — works sometimes, but fragile; the backup direction may itself be blocked

**Research questions:**
- What is the canonical Nav2 approach for recovering when the robot's start cell is in lethal/near-lethal space?
- Is there a SmacPlanner2D tolerance setting that allows planning from a near-lethal start?
- Should the agent validate goal cells against the costmap (not just the occupancy grid) before sending them to Nav2?
- What is the correct sequence for cost-aware recovery: clear costmap → move → replan, or move → clear → replan?
- Does Nav2 have a built-in "recover from occupied start" behavior in its BT action library (Jazzy)?
- How should the inflation radius and robot radius be balanced so the robot never lands in lethal space after normal navigation?

---

### 2. MPPI controller can't navigate through narrow doors (0.06m margin)

With inflation_radius=0.25m and robot_radius=0.22m, a 1m door has only 0.06m (~1.2 cells at 5cm) of traversable width. MPPI's stochastic trajectory sampling rarely finds a valid trajectory through this gap. The old Nav2 recovery BT worked around this by physically spinning/backing up the robot until it accidentally threaded through, but this is unreliable and expensive.

**Research questions:**
- What are the recommended MPPI parameters (`num_trajectories`, `trajectory_for_searching`, sampling resolution, `w_obstacle`, `obstacle_cost_weight`) for navigating reliably through tight spaces?
- Is there a way to tell MPPI to bias its sampling toward the center of narrow passages (e.g., using the signed-distance field of the costmap)?
- Should the inflation radius be reduced for the local costmap (MPPI obstacle avoidance) while keeping it higher for the global costmap (path planning)?  What are the tradeoffs?
- Is there a MPPI variant or DiffDrive model configuration that handles tight corridors better?
- What is the recommended Nav2 Jazzy approach for environments with 1m doors and a 0.44m robot (DiffDrive)?
- Are there alternative local planners (e.g., TEB, GRACE, regulated pure pursuit) that handle narrow passages more reliably than MPPI?

---

### 3. collision_monitor loses bond heartbeat under CPU/DDS load → Nav2 stack dies

Under sustained CPU/DDS load (e.g., rapid costmap clearing loops, high-frequency MPPI planning through complex areas), the `collision_monitor` node stops sending bond heartbeats to the `lifecycle_manager`. After the bond timeout (currently 60s), the lifecycle_manager shuts down the ENTIRE Nav2 stack. All subsequent goals are rejected as "Action server is inactive" — the robot is stuck forever.

`attempt_respawn_on_failure: true` is set, but it's unclear whether the respawn path is clean (re-activates all nodes correctly, doesn't leave stale state).

**Research questions:**
- What is the recommended `bond_timeout` and `bond_heartbeat_period` for a loaded single-GPU system running Gazebo + SLAM + Nav2 simultaneously?
- Does `attempt_respawn_on_failure: true` in Nav2 Jazzy actually work reliably? What are known failure modes of the respawn path?
- How should the agent detect that Nav2 has gone down (vs. a goal being legitimately rejected)? What is the correct client-side wait/retry pattern after a lifecycle event?
- Is there a way to make `collision_monitor` more resilient to CPU starvation (e.g., raising its thread priority, using a separate DDS participant, reducing its update rate)?
- Should `collision_monitor` be in a separate lifecycle group from the navigation nodes so its death doesn't kill the planner/controller?
- What is the recommended Nav2 Jazzy architecture for a system where individual nodes may transiently fail?

---

### 4. Nav2 BT recovery behaviors cause DDS flooding

The default Nav2 recovery BT (`navigate_to_pose_w_replanning_and_recovery.xml`) enters a rapid ClearGlobalCostmap → replan loop on planning failure. This generates enough DDS traffic to starve `collision_monitor`'s heartbeat (see problem 3).

We switched to `navigate_w_replanning_only_if_path_becomes_invalid.xml` (no recovery) to stop the storm, but lost the ability to recover from local minima (door navigation, "Start occupied").

**Research questions:**
- What is the correct Nav2 Jazzy BT design for recovery that limits costmap-clearing to a maximum of 1–2 retries without looping?
- Is there a standard BT node that throttles recovery frequency (e.g., "don't clear costmap more than once per 10s")?
- What recovery sequence (spin → backup → clear local → clear global) is most effective for getting a stuck robot moving again, and in what order?
- How can BT recovery be made robust without generating DDS flooding? Is there a recommended pattern for async/rate-limited recovery actions?
- Are there Nav2 Jazzy BT nodes or plugins specifically designed for "robot is in occupied space" recovery?

---

### 5. Generic "robot stuck or in bad space" recovery strategy

The underlying need: when the robot is stuck for any reason (obstacle fallen on it, ended up in inflation zone, MPPI local minimum, costmap stale), it should autonomously recover and continue exploration — not get stuck forever. We want this to be generic, not a brittle special case.

**Research questions:**
- What is the recommended Nav2 Jazzy pattern for a fully autonomous recovery loop that handles: (a) robot in lethal costmap space, (b) MPPI stuck in local minimum, (c) Nav2 lifecycle failure/respawn?
- How do production Nav2 deployments (e.g., warehouse robots, delivery robots) handle these failure modes? Is there published work or open-source examples?
- What is the difference between Nav2's `ClearCostmapExceptRegion` vs `ClearEntireCostmap`, and when should each be used for recovery?
- Is there a Nav2 recovery behavior that specifically handles "robot is in or near lethal space" by computing a small escape motion?
- Should the agent maintain a "recovery state machine" independent of the BT (at the ROS action client level), or should all recovery logic live in the BT?

---

## What we want from the research

1. **Specific Nav2 Jazzy (2024–2025) configuration** recommendations with parameter names and values, not general advice.
2. **BT XML patterns** for robust recovery with bounded retry counts and no DDS flooding.
3. **Known working architectures** for narrow-corridor navigation with MPPI — real deployments or research papers.
4. **The canonical recovery sequence** for "robot in occupied/lethal space" in Nav2.
5. **Lifecycle resilience patterns** — how to structure Nav2 nodes so a single node failure doesn't kill the whole stack.

Prefer 2024–2025 sources. Quote exact parameter names from Nav2 Jazzy documentation or source code where possible.
