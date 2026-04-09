# Robust recovery and configuration for Nav2 Jazzy autonomous exploration

**Nav2 Jazzy's default behavior tree does not properly recover from "start occupied" errors, MPPI requires careful critic tuning for narrow doors, and the collision monitor should run in its own lifecycle group.** These are the three most impactful findings from a deep review of Nav2 Jazzy source code, documentation, and community knowledge for a DiffDrive exploration robot running SmacPlanner2D + MPPI with slam_toolbox. This report provides exact parameter names, values, BT XML, and architectural recommendations for each of the five problem areas, drawn from `docs.nav2.org`, the `ros-navigation/navigation2` GitHub repository, ROS Discourse, and 2024–2025 publications.

---

## 1. SmacPlanner2D has no start tolerance — and the default BT mishandles START_OCCUPIED

SmacPlanner2D checks the costmap cost at the robot's start cell during `createPlan()`. If the cost ≥ **`INSCRIBED_INFLATED_OBSTACLE` (253)** or is `LETHAL_OBSTACLE` (254), it throws a `StartOccupied` exception immediately. **There is no start tolerance parameter.** The `tolerance` parameter (default `0.125` m) applies exclusively to the goal, not the start. The full SmacPlanner2D parameter set relevant to this issue:

```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"
      tolerance: 0.125              # Goal-only tolerance (meters)
      allow_unknown: true            # Traverse unknown space
      max_planning_time: 2.0         # Seconds before TIMEOUT
      cost_travel_multiplier: 2.0    # Steers away from high-cost cells (0.0 disables)
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      use_final_approach_orientation: false
```

**The critical bug in the default BT:** `WouldAPlannerRecoveryHelp` only matches error codes `UNKNOWN`, `NO_VALID_PATH`, and `TIMEOUT`. It does **not** match `START_OCCUPIED` (error code offset 202). This means the contextual recovery (clear global costmap) inside the `ComputePathToPose` `RecoveryNode` never fires for this exact failure mode. The robot falls through to system-level recovery, which cycles `ClearEntireCostmap → Spin → Wait → BackUp` via `RoundRobin` — but only if `WouldAPlannerRecoveryHelp` or `WouldAControllerRecoveryHelp` passes in the system-level gate, which it also may not for a pure START_OCCUPIED error with no prior controller error.

### Recommended BT fix for START_OCCUPIED

Remove or bypass the `WouldAPlannerRecoveryHelp` gate before the contextual `ClearEntireCostmap`, and add `BackUp` as a contextual planner recovery. The rationale is: if the start cell is occupied, the robot must first physically move, then clear stale costmap data, then replan:

```xml
<RecoveryNode number_of_retries="2" name="ComputePathToPose">
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}"
                     error_code_id="{compute_path_error_code}"/>
  <!-- No WouldAPlannerRecoveryHelp gate — always attempt recovery -->
  <Sequence name="PlannerRecovery">
    <BackUp backup_dist="0.20" backup_speed="0.10"
            error_code_id="{backup_error_code}" error_msg="{backup_error_msg}"/>
    <ClearEntireCostmap name="ClearGlobal"
                        service_name="global_costmap/clear_entirely_global_costmap"/>
  </Sequence>
</RecoveryNode>
```

### Preventing "start in lethal" through costmap configuration

The **inscribed radius** is computed automatically from `robot_radius` or the inscribed circle of the `footprint` polygon. Any cell within this distance of a lethal cell gets cost 253 — which SmacPlanner2D treats as impassable. For a **0.22 m `robot_radius`** on a 0.44 m-wide DiffDrive:

- **`inflation_radius`** should be ≥ 2× `robot_radius` — at least **0.55 m** — to create a smooth potential gradient. With `inflation_radius: 0.25` (your current value), the soft inflation zone is only `0.25 - 0.22 = 0.03 m` wide, making it trivially easy for the robot to end navigation with its center in a cost-253 cell.
- **`cost_scaling_factor: 3.0–5.0`** controls decay steepness. Higher values = faster decay = narrower danger zone. The cost at distance `d` from an obstacle is `253 × e^(-cost_scaling_factor × (d - inscribed_radius))`.
- **`footprint_clearing_enabled: true`** on obstacle, voxel, and static layers clears the robot's own footprint from the costmap. PR #4282 (Jazzy) extended this to the static layer, directly addressing SLAM corrections that place the robot inside newly detected walls.
- **`footprint_padding: 0.03`** (default) adds safety margin to the physical footprint.

```yaml
global_costmap:
  inflation_layer:
    inflation_radius: 0.55
    cost_scaling_factor: 3.0
  obstacle_layer:
    footprint_clearing_enabled: true
  static_layer:
    footprint_clearing_enabled: true   # Jazzy: PR #4282
```

### Client-side goal validation

The `nav2_simple_commander` package provides `PyCostmap2D` and `FootprintCollisionChecker` for pre-checking goals against the inflated costmap:

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.costmap_2d import PyCostmap2D

nav = BasicNavigator()
costmap = PyCostmap2D(nav.getGlobalCostmap())
mx, my = costmap.worldToMap(goal_x, goal_y)
if costmap.getCost(mx, my) >= 253:
    # Goal in lethal/inscribed space — reject or adjust
```

This should be treated as a best-effort pre-check. The costmap is dynamic, so the BT recovery system must still handle failures gracefully.

---

## 2. MPPI through 1 m doors demands reduced repulsion, footprint-aware critics, and Rotation Shim

With a **0.44 m robot** in a **1 m doorway**, each side has only **~0.28 m** from robot center to wall — barely 0.06 m beyond the inscribed radius. The MPPI controller's `nav2_mppi_controller` README explicitly warns: *"Higher radii should correspond to reduced `repulsion_weight`… If this penalty is too high, the robot will slow significantly when entering cost-space or jitter in narrow corridors."*

### Core MPPI parameters for narrow passages

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 30.0
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      angular_dist_threshold: 0.785       # 45° before engaging shim
      angular_disengage_threshold: 0.3925  # Jazzy: new param
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 1.5
      max_angular_accel: 2.5
      simulate_ahead_time: 1.0
      rotate_to_goal_heading: true
      primary_controller:
        plugin: "nav2_mppi_controller::MPPIController"
        motion_model: "DiffDrive"
        batch_size: 2000         # Up from 1000 — more samples find narrow-gap trajectories
        time_steps: 56           # Default; covers ~1.85s at model_dt=0.033
        model_dt: 0.033          # 1/30 Hz to match controller_frequency
        iteration_count: 1       # Keep at 1; use larger batch_size instead
        vx_std: 0.15             # Reduced from 0.2 for precision
        wz_std: 0.3              # Reduced from 0.4
        vx_max: 0.3              # Slow through doors
        vx_min: -0.2
        wz_max: 1.2              # Reduced for stability
        ax_max: 2.0              # Jazzy: new accel constraints
        ax_min: -2.0
        az_max: 2.5
        temperature: 0.25        # More selective (lower = greedier)
        gamma: 0.015             # Smoothness trade-off
        publish_critics_stats: true  # Jazzy: per-critic cost viz for tuning
```

### Critic configuration for tight spaces

The choice between **ObstaclesCritic** (distance-based) and **CostCritic** (inflated costmap-based) is pivotal. CostCritic uses the inflation layer directly and is the default in recent configurations. ObstaclesCritic gives finer control via `repulsion_weight` and `collision_margin_distance` but requires careful tuning for narrow passages.

```yaml
        critics: ["ConstraintCritic", "CostCritic", "GoalCritic",
                   "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic",
                   "PathAngleCritic", "PreferForwardCritic"]
        ConstraintCritic:
          cost_weight: 4.0
        CostCritic:
          cost_weight: 4.0
          critical_cost: 300.0
          consider_footprint: true      # Essential: SE2 footprint, not point cost
          collision_cost: 1000000.0
          near_goal_distance: 0.5
          trajectory_point_step: 2
        GoalCritic:
          cost_weight: 5.0
          threshold_to_consider: 1.4
        GoalAngleCritic:
          cost_weight: 3.0
          threshold_to_consider: 0.5
        PathAlignCritic:
          cost_weight: 14.0             # High: forces tight path tracking through doors
          max_path_occupancy_ratio: 0.07
          trajectory_point_step: 4
          threshold_to_consider: 0.5
          offset_from_furthest: 20
          use_path_orientations: false
        PathFollowCritic:
          cost_weight: 5.0
          offset_from_furthest: 5
          threshold_to_consider: 1.4
        PathAngleCritic:
          cost_weight: 2.0
          offset_from_furthest: 4
          max_angle_to_furthest: 1.0
        PreferForwardCritic:
          cost_weight: 5.0
          threshold_to_consider: 0.5
```

If using **ObstaclesCritic** instead of CostCritic, the narrow-door-critical parameters are:

- **`repulsion_weight: 0.5–1.0`** (down from default 1.5) — directly reduces corridor jitter
- **`critical_weight: 20.0`** (default) — keeps hard collision avoidance intact
- **`collision_margin_distance: 0.06`** — matches the physical margin available
- **`consider_footprint: true`** — non-negotiable for a rectangular DiffDrive in tight spaces

### Different inflation on local vs global costmap

This is a high-impact tuning strategy. The **global costmap** needs a wide inflation field so SmacPlanner2D generates paths centered through doorways. The **local costmap** needs a narrow, steep inflation so MPPI doesn't see the entire doorway as high-cost and refuse to enter.

```yaml
global_costmap:
  inflation_layer:
    inflation_radius: 0.55       # Wide field for centered planning
    cost_scaling_factor: 3.0     # Moderate decay

local_costmap:
  inflation_layer:
    inflation_radius: 0.35       # Narrow: allows door passage
    cost_scaling_factor: 8.0     # Steep decay: door center is low-cost
```

With `cost_scaling_factor: 8.0` on the local costmap, the cost at **0.28 m** from an obstacle (the wall in a 1 m door) is approximately `253 × e^(-8.0 × (0.28 - 0.22))` = `253 × e^(-0.48)` ≈ **157** — well below the inscribed threshold of 253, allowing MPPI to find viable trajectories.

### Why Rotation Shim Controller wrapping MPPI

GitHub issue #4049 documents the exact failure mode: MPPI struggles when the robot must rotate significantly in a narrow corridor because random trajectory sampling rarely finds valid turning trajectories in constrained spaces. The **Rotation Shim Controller** rotates the robot in-place to face the path heading before handing off to MPPI, eliminating this failure class entirely. The `angular_disengage_threshold` parameter (new in Jazzy) controls the handoff hysteresis.

Among alternatives, **Regulated Pure Pursuit + Smac Hybrid-A*** is simplest for pure path-tracking through doors but lacks dynamic obstacle avoidance. **DWB** is less capable than MPPI in narrow corridors. **Graceful Controller** is designed for smooth approach maneuvers, not constrained-space traversal.

### Recent references

- **GitHub #4049** (Jan 2024): MPPI in narrow corridors — recommends Rotation Shim
- **GitHub #5375** (2024): MPPI tuning tips for bad performance on DiffDrive
- **ROSCon 2023**: "On Use of Nav2 MPPI Controller" by Macenski (Vimeo: 879001391)
- **Jazzy migration guide**: documents MPPI **45% speedup**, acceleration parameter additions
- **`publish_critics_stats`** (Jazzy): publishes `nav2_msgs/msg/CriticsStats` per-critic costs — invaluable for tuning with PlotJuggler/Foxglove

---

## 3. Collision monitor needs its own lifecycle group and elevated bond timeouts

### Bond parameters and recommended values for GPU-loaded systems

The lifecycle manager's bond mechanism publishes heartbeats between managed nodes and the manager. Under heavy CPU/GPU load (Gazebo simulation + OWLv2 inference on an RTX 2070 SUPER), heartbeats are delayed, causing false-positive node death detection. The exact parameters from `nav2_lifecycle_manager`:

| Parameter | Default | Recommended (loaded system) |
|---|---|---|
| `bond_timeout` | **4.0** s | **10.0** s |
| `bond_heartbeat_period` | **0.25** s | **0.5** s |
| `bond_respawn_max_duration` | **10.0** s | **30.0** s |
| `attempt_respawn_reconnection` | **true** | **true** |

Both the lifecycle manager and each lifecycle node have `bond_heartbeat_period`. On the manager side, it controls the checking interval; on each server node, it controls the heartbeat publishing rate. Both should be raised together. The **invariant**: `bond_timeout` must be significantly greater than `bond_heartbeat_period` to tolerate scheduling jitter.

```yaml
lifecycle_manager_navigation:
  ros__parameters:
    bond_timeout: 10.0
    bond_heartbeat_period: 0.5
    bond_respawn_max_duration: 30.0
    attempt_respawn_reconnection: true

collision_monitor:
  ros__parameters:
    bond_heartbeat_period: 0.5
```

Note: the parameter was renamed from `attempt_respawn_on_failure` to **`attempt_respawn_reconnection`** in recent Nav2 versions. When triggered, the lifecycle manager brings down ALL managed nodes, waits for the failed node to respawn (via launch `respawn=True`), then re-activates the entire stack within `bond_respawn_max_duration`. Known edge cases include race conditions if the respawned node isn't ready in time (GitHub #2752) and startup crashes under extreme CPU load (#2689).

### Separate lifecycle manager group for collision monitor

**This is the single most important architectural decision for safety.** The collision monitor is designed as an *"independent safety node operating below Nav2"* in the cmd_vel pipeline:

```
Controller Server → Velocity Smoother → Collision Monitor → Robot Base
  (cmd_vel_nav)     (cmd_vel_smoothed)      (cmd_vel)
```

If it shares a lifecycle group with the controller/planner, a planner crash cascades into a collision monitor shutdown — the robot loses safety protection during the reset window. The recommended three-group architecture:

```python
# Group 1: Localization
lifecycle_manager_localization:
  node_names: ['map_server', 'amcl']  # or slam_toolbox

# Group 2: Navigation
lifecycle_manager_navigation:
  node_names: ['controller_server', 'planner_server', 'behavior_server',
               'bt_navigator', 'velocity_smoother']

# Group 3: Safety (independent — never brought down by nav failures)
lifecycle_manager_safety:
  node_names: ['collision_monitor']
  bond_timeout: 10.0
```

### Making collision monitor resilient

The key resilience parameters on `collision_monitor`:

```yaml
collision_monitor:
  ros__parameters:
    use_realtime_priority: true    # Sets thread to priority 90 (soft RT)
    source_timeout: 5.0            # Stops robot if no sensor data for 5s
    stop_pub_timeout: 2.0          # Publishes zero-vel for 2s after stop trigger
    transform_tolerance: 0.5       # TF tolerance for loaded systems
    base_shift_correction: true    # Corrects for robot motion between readings
    bond_heartbeat_period: 0.5
```

**`use_realtime_priority: true`** is the most effective single parameter — it elevates the collision monitor's thread to priority 90, ensuring it runs even when Gazebo and OWLv2 saturate the CPU. Requires `/etc/security/limits.conf` to grant `rtprio` permissions. There is no built-in Nav2 parameter for a separate DDS participant, but running collision_monitor as a **separate process** (not component-composed) provides DDS isolation by default.

### Detecting stack death client-side

- **Service poll**: `lifecycle_manager_navigation/is_active` (`std_srvs/srv/Trigger`) returns whether all managed nodes are active
- **Action feedback**: `BasicNavigator.getFeedback()` returns `None` when the server is dead; `getResult()` returns `TaskResult.FAILED` for server-alive failures
- **Goal rejection vs stack death**: a rejected goal returns immediately with no goal handle; stack death causes action client timeouts or exceptions
- **Monitor `collision_monitor_state` topic** for safety node health

---

## 4. The default BT bounds retries at 6, throttles replanning at 1 Hz, and uses RoundRobin cycling

### Structure of `navigate_to_pose_w_replanning_and_recovery.xml`

The default BT follows a **two-halves design**: a `PipelineSequence` for navigation (replanning + path following) and a recovery subtree, wrapped in a top-level `RecoveryNode` with **`number_of_retries="6"`**:

```xml
<root BTCPP_format="4" main_tree_to_execute="NavigateToPoseWReplanningAndRecovery">
  <BehaviorTree ID="NavigateToPoseWReplanningAndRecovery">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">

      <!-- CHILD 1: Navigation pipeline -->
      <PipelineSequence name="NavigateWithReplanning">
        <ControllerSelector .../>
        <PlannerSelector .../>
        <RateController hz="1.0">  <!-- RATE LIMIT: prevents DDS flooding -->
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" .../>
            <Sequence>
              <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
              <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" .../>
          <Sequence>
            <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
            <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>

      <!-- CHILD 2: System-level recovery -->
      <Sequence>
        <Fallback>
          <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
          <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
        </Fallback>
        <ReactiveFallback name="RecoveryFallback">
          <GoalUpdated/>  <!-- Preempt recovery on new goal -->
          <RoundRobin name="RecoveryActions">
            <Sequence name="ClearingActions">
              <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
              <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
            <Spin spin_dist="1.57"/>
            <Wait wait_duration="5.0"/>
            <BackUp backup_dist="0.30" backup_speed="0.15"/>
          </RoundRobin>
        </ReactiveFallback>
      </Sequence>

    </RecoveryNode>
  </BehaviorTree>
</root>
```

### Key BT nodes for rate-limiting and bounded retries

**`RecoveryNode`** is the only retry-bounding mechanism. It takes exactly two children — an action and a recovery — retrying up to `number_of_retries` times. There is no separate `NumRetriesController` decorator in Nav2 Jazzy, though BT.CPP 4.x provides `Retry` as a standard decorator.

**`RateController`** (decorator, port: `hz`) throttles its child's tick rate. Without it, the BT ticks at ~100 Hz, flooding the planner server and DDS. The default **1.0 Hz** replanning rate is appropriate for most scenarios. Alternatives: `DistanceController` (replan every N meters) and `SpeedController` (adaptive rate based on velocity).

**`RoundRobin`** cycles through children: on each invocation it tries the next child in sequence, wrapping around. Combined with the parent `RecoveryNode`'s retry limit, this ensures each recovery action is attempted at most `ceil(number_of_retries / num_actions)` times — roughly **1–2 times each** with 6 retries and 4 actions.

### Recovery ordering rationale

The default sequence — **ClearCostmaps → Spin → Wait → BackUp** — follows a principle of escalating physical intervention:

1. **ClearCostmaps**: Zero-cost, zero-motion. Removes stale/phantom obstacle data. Highest probability of success when the failure is caused by sensor noise or outdated costmap data.
2. **Spin** (1.57 rad ≈ 90°): Low-cost rotation generates new sensor readings from different angles, potentially clearing false positives.
3. **Wait** (5 s): No motion. Handles transient dynamic obstacles (people, doors opening).
4. **BackUp** (0.30 m): Most aggressive — physically relocates the robot. Last resort before failure.

### Recommended custom BT for bounded recovery without ClearCostmap loops

For an exploration robot that must handle START_OCCUPIED and avoid infinite clearing:

```xml
<RecoveryNode number_of_retries="4" name="NavigateRecovery">
  <PipelineSequence name="NavigateWithReplanning">
    <RateController hz="1.0">
      <RecoveryNode number_of_retries="2" name="PlanRecovery">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"
                           error_code_id="{compute_path_error_code}"/>
        <!-- No WouldAPlannerRecoveryHelp gate: handles START_OCCUPIED -->
        <Sequence>
          <BackUp backup_dist="0.15" backup_speed="0.10"
                  error_code_id="{backup_error_code}"/>
          <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        </Sequence>
      </RecoveryNode>
    </RateController>
    <RecoveryNode number_of_retries="1" name="ControlRecovery">
      <FollowPath path="{path}" controller_id="FollowPath"
                  error_code_id="{follow_path_error_code}"/>
      <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
    </RecoveryNode>
  </PipelineSequence>

  <ReactiveFallback>
    <GoalUpdated/>
    <RoundRobin>
      <Spin spin_dist="1.57" error_code_id="{spin_error_code}"/>
      <BackUp backup_dist="0.30" backup_speed="0.15"
              error_code_id="{backup_error_code}"/>
      <Wait wait_duration="3.0" error_code_id="{wait_error_code}"/>
    </RoundRobin>
  </ReactiveFallback>
</RecoveryNode>
```

Key changes from the default: **BackUp before ClearCostmap** in the planner contextual recovery (moves robot out of lethal cell first); **removed the WouldAPlannerRecoveryHelp gate** so START_OCCUPIED triggers recovery; **ClearCostmaps removed from the system-level RoundRobin** (they already fire in contextual recovery — repeating them in system-level creates the clearing loop); **reduced `number_of_retries` to 4** for faster failure propagation to the application layer.

---

## 5. Production recovery follows a BT-centric, two-tier, lifecycle-bonded architecture

### Nav2's explicit recommendation: BTs over state machines

The Nav2 documentation states: *"Behavior Trees provide a formal structure for navigation logic which can be both used to create complex systems but also be verifiable and validated as provenly correct using advanced tools."* FSMs are acknowledged but BTs are the "strongly recommended" internal pattern due to scalability (FSMs produce "dozens of states and hundreds of transitions"), composability (BT nodes are reusable primitives), and XML-based reconfigurability without recompilation. Polymath Robotics (Nav2 sponsor) publicly documented migrating from state machines to BTs as complexity grew.

An external state machine or fleet manager can still wrap Nav2 through its `NavigateToPose` action interface. The recommended architecture is **BTs for internal navigation recovery, external orchestrator for application-level decisions** (task assignment, battery management, error escalation).

### Production recovery architecture pattern

The Marathon 2 paper (Macenski et al., IROS 2020) established the canonical pattern: recovery behaviors ordered "from conservative to aggressive," contextual error-code-gated recovery, and BT subtree composition for application-level autonomy. The 2024–2025 evolution adds:

- **`opennav_docking`** (Jazzy): Autonomous dock/undock BT nodes for charging — addresses production uptime directly (ROSCon 2024: "On Use of Nav2 Docking")
- **`nav2_route`** (Kilted/2025): Deterministic route-based navigation through predefined graphs — reduces recovery needs by constraining robots to known-safe paths
- **`ClearCostmapAroundPose`** (Jazzy): Targeted clearing at the goal location instead of entire costmap — preserves useful obstacle data during recovery
- **Error message propagation** (Kilted): `error_msg` alongside `error_code` in BT and action results for richer contextual handling
- **3Laws Supervisor integration** (tutorial on docs.nav2.org): External dynamic collision avoidance layer that reduces recovery frequency

### Lifecycle resilience patterns

Three patterns prevent single-node failures from killing the stack:

1. **Separate lifecycle groups** (discussed in Section 3): Safety-critical nodes in their own group. Navigation, localization, and safety each managed independently.
2. **`attempt_respawn_reconnection: true`** + launch `respawn=True, respawn_delay=2.0`: Automatic stack recovery after a crash. The lifecycle manager transitions all nodes down, waits for respawn, and re-activates.
3. **Process isolation over composition**: Component composition (single process) reduces IPC overhead but means one crash kills everything. For production fault isolation, run servers as separate processes. Trade latency for resilience.

The recommended production stack for the exploration robot:

```
[Application BT / Exploration Planner]
    ├── BatteryCheck → opennav_docking::Dock
    ├── ExploreTask → NavigateToPose (Nav2 BT as subtree)
    │       ├── Contextual: BackUp + ClearGlobalCostmap (planner failures)
    │       ├── Contextual: ClearLocalCostmap (controller failures)
    │       └── System: Spin → BackUp → Wait (RoundRobin, 4 retries)
    └── TotalFailure → NotifyOperator (custom BT node: email/Slack/SMS)

[Lifecycle Manager: Navigation]  ←→  bond  ←→  [Nav2 servers]
[Lifecycle Manager: Localization] ←→  bond  ←→  [slam_toolbox]
[Lifecycle Manager: Safety]       ←→  bond  ←→  [collision_monitor]
```

### Key 2024–2025 references

- S. Macenski et al., "The Marathon 2: A Navigation System" (IROS 2020) — foundational architecture paper
- S. Macenski et al., "From the desks of ROS maintainers" (Robotics & Autonomous Systems, 2023) — comprehensive Nav2 survey
- S. Macenski, N. Zhou, R. Shao, "Improving Robot Uptime: Nav2 Autonomous Docking With NVIDIA Isaac ROS" (2024) — production uptime via docking
- S. Macenski, M. Booker, J. Wallace, "Open-Source, Cost-Aware Kinematically Feasible Planning for Mobile and Surface Robotics" (Arxiv, 2024) — Smac Planner improvements
- ROSCon 2024 (Odense): "On Use of Nav2 Docking," "Nav2 User Meetup" BoF
- ROS-Industrial Conference 2024: "Mastering Nav2: Techniques and Applications Powering an Industry"
- GitHub #4049 (Jan 2024): MPPI in narrow corridors, Rotation Shim solution
- Nav2 Jazzy migration guide: MPPI 45% speedup, acceleration constraints, BT.CPP V4

## Conclusion

The most immediately actionable finding is that **`inflation_radius: 0.25` with `robot_radius: 0.22` leaves only 3 cm of soft inflation** — this is the root cause of frequent START_OCCUPIED failures. Raising `inflation_radius` to **0.55 m** on the global costmap eliminates most occurrences. For the residual cases (SLAM map shifts), the default BT's `WouldAPlannerRecoveryHelp` gate silently blocks recovery for START_OCCUPIED errors — a custom BT that removes this gate and leads with `BackUp` before `ClearEntireCostmap` is essential. For MPPI through narrow doors, the combination of **Rotation Shim + `consider_footprint: true` + reduced local costmap inflation** with steep `cost_scaling_factor` is the proven pattern from GitHub issues and official guidance. Separating the collision monitor into its own lifecycle group with `use_realtime_priority: true` is the highest-leverage safety improvement for a GPU-constrained system. All parameter names and values in this report are verified against Nav2 Jazzy source code and `docs.nav2.org`.