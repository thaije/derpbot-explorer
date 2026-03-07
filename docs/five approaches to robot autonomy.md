# Five architectures for autonomous exploration in ROS 2 Jazzy

**A differential-drive robot with 2D LiDAR, RGBD camera, and two 8GB GPUs can achieve fully autonomous indoor exploration, mapping, and open-vocabulary object detection today using any of five distinct architectures**, each trading off maturity, intelligence, and computational cost differently. The classical modular pipeline (slam_toolbox + Nav2 + YOLO-World) is production-ready and runs comfortably on this hardware. At the frontier, LLM-orchestrated systems and semantic scene graphs offer dramatically smarter exploration but introduce latency and complexity. The most important finding across all five approaches: **open-vocabulary detection has become cheap enough (~1–3 GB VRAM, 50+ FPS) that the real differentiator is exploration strategy, not perception**. YOLOE26 or YOLO-World provide real-time open-vocabulary detection well within the 8GB budget, leaving substantial GPU headroom for learned planners, VLMs, or scene graph construction.

This report evaluates each architecture against the DerpBot's exact sensor suite (360° 2D LiDAR with 720 rays, 640×480 RGBD camera with 90° HFOV, IMU, wheel odometry) and dual-GPU constraint, providing specific packages, model sizes, and inference speeds for practical implementation in ROS 2 Jazzy.

---

## Approach 1: the classical modular pipeline remains the gold standard

The classical stack — slam_toolbox for SLAM, Nav2 for planning/control, frontier-based exploration, and a GPU-accelerated object detector — represents the most mature and reliable architecture. Every component has been battle-tested on hundreds of production robots.

**SLAM: slam_toolbox dominates.** Released for Jazzy as v2.8.3 (`ros-jazzy-slam-toolbox`), maintained by Steve Macenski, with **2,300+ GitHub stars**. Online async mode is recommended for real-time exploration — it drops scans gracefully under load rather than falling behind. Performance benchmarks show **5× real-time for spaces up to ~30,000 sq ft** and successful mapping of 200,000+ sq ft buildings. Runs entirely on CPU with negligible overhead (~5% of a single core for indoor environments). Google Cartographer is technically available on Jazzy but effectively unmaintained by Google; the ROS 2 fork receives only build fixes. RTAB-Map (v0.22.1 on Jazzy) is viable if RGBD-fused 3D mapping is desired, but adds unnecessary complexity for 2D exploration.

**Navigation: Nav2 with MPPI controller.** Nav2 for Jazzy ships with the **MPPI controller running 45% faster** than in Iron, explicit differential-drive motion models, and the new Graceful Controller. For DerpBot, the recommended configuration is SmacPlanner2D (cost-aware A*, cleaner paths than NavFn, ~40ms planning time) paired with MPPI (model-predictive sampling controller, runs at **100+ Hz on modest CPUs**, best dynamic obstacle avoidance among Nav2 controllers). The Rotation Shim Plugin handles initial heading alignment. Recovery behaviors — Spin, BackUp, Wait — are plugin-based and execute via a RoundRobin fallback sequence, retrying up to 6 times before declaring failure.

**Frontier exploration: build your own.** No official Nav2 exploration plugin exists. The community package `m-explore-ros2` (252 GitHub stars) provides a ROS 2 port of explore_lite but lacks a Jazzy binary release and uses greedy nearest-frontier selection. For time-limited missions, a **custom frontier node** is strongly recommended: subscribe to `/map`, detect free cells adjacent to unknown cells via BFS, cluster frontier cells, score by information-gain-to-travel-cost ratio, and send goals to Nav2's `NavigateToPose` action server. This node runs on CPU at 0.2–1.0 Hz and interfaces cleanly with Nav2 without any plugin registration.

**Detection: one GPU, plenty of headroom.** YOLO-World-S (13M parameters, **74 FPS on V100, ~1–2 GB VRAM**) or the newer YOLOE26-S (29.9% LVIS mAP, **161 FPS on T4, ~1–2 GB VRAM**) runs on one GPU while the entire classical stack runs on CPU. The second GPU sits idle or handles CLIP embeddings for richer semantic features. ByteTrack adds persistent tracking IDs at zero GPU cost (CPU-only Kalman filter + Hungarian algorithm). The detection node subscribes to `/camera/color/image_raw`, publishes `vision_msgs/Detection2DArray`, and operates completely independently from Nav2.

**Generalizability and robot-agnosticism** are strong. Nav2 supports differential, omnidirectional, Ackermann, and legged platforms through configurable motion models. slam_toolbox works with any 2D LiDAR providing `sensor_msgs/LaserScan`. The architecture transfers to any ground robot with a LiDAR; drones require 3D SLAM and different planners. Moving to a different indoor environment requires zero reconfiguration.

**Computational budget:**

| Component | Runs on | Resource usage |
|-----------|---------|---------------|
| slam_toolbox (async) | CPU | ~5% single core |
| Nav2 (costmap + SmacPlanner2D + MPPI) | CPU | <20% core at 30 Hz |
| Custom frontier detector | CPU | <5% core at 1 Hz |
| YOLOE26-S + ByteTrack | GPU 0 | ~1–2 GB VRAM, 50+ FPS |
| **GPU 1** | **Idle** | **8 GB available** |

---

## Approach 2: behavior trees orchestrate a smarter mission loop

The second approach keeps the same Nav2 backend but replaces the simple frontier script with a **BehaviorTree.CPP (v4.5+) mission tree** that orchestrates explore-detect-report cycles with explicit time management, parallel detection monitoring, and structured recovery.

**BT.CPP in Nav2 Jazzy** upgraded from v3.8 to v4.5, requiring BT XML format version 4 and handling of the new `SKIPPED` status. Nav2 ships with a comprehensive library of BT nodes: `ComputePathToPose`, `FollowPath`, `Spin`, `BackUp`, `Wait`, `TimeExpired`, `GoalUpdated`, `RateController`, and ~30 others. The `PipelineSequence` control node enables replanning during path following, and `RecoveryNode` implements retry-with-recovery patterns. The key architectural advantage: **BTs compose naturally**, allowing the exploration logic to be loaded as a subtree within a higher-level mission tree.

The recommended BT structure for explore-detect-report uses a `ReactiveSequence` root guarded by `TimeExpired` to enforce the mission deadline. Inside, a `ReactiveFallback` checks whether all frontiers are exhausted (mission complete) or proceeds with exploration. The exploration branch uses a `Parallel` node to run navigation and detection monitoring concurrently — the detection condition node subscribes to the `/detections` topic and triggers reporting whenever new objects appear, without interrupting navigation. At each frontier, a `Spin` action (full 360° rotation) maximizes camera coverage given the 90° HFOV.

**Custom BT nodes** for this architecture are straightforward to implement. A `SelectNextFrontier` action node reads the `/map` occupancy grid, computes frontiers, applies scoring, and writes the goal to the blackboard. A `CheckForDetections` condition node wraps a subscription to `vision_msgs/Detection2DArray` and returns SUCCESS when detections are present. The `BehaviorTree.ROS2` wrapper package (`RosTopicSubNode<>`, `RosActionNode<>`) reduces boilerplate for ROS-topic-based conditions and action-server-based behaviors.

**py_trees** (v2.4.0, `ros-jazzy-py-trees`) offers a Python alternative for teams preferring rapid prototyping. It runs independently from Nav2's C++ BT system but can call Nav2 action servers as a higher-level orchestrator. The `nav2_simple_commander` Python API (`BasicNavigator` class with `goToPose()`, `followWaypoints()`) provides an even simpler programmatic interface for teams that find BT XML too verbose.

**Costmap integration** for detection awareness uses Nav2's plugin-based costmap layer system. A custom layer extending `nav2_costmap_2d::Layer` subscribes to detection topics and marks cells around detected objects with costs — useful for avoiding obstacles the LiDAR misses (e.g., glass, thin objects detected by camera). Setting `track_unknown_space: true` is essential for exploration to distinguish unexplored from free space. Costmap filters can mark visited areas with higher cost to bias the planner toward unvisited regions.

**Where this shines over Approach 1:** explicit time management via `TimeExpired`, structured recovery with configurable retry limits, concurrent detection monitoring without polling, and composable mission logic that's easy to extend (add "return to base" as a subtree, add "report via radio" as an action node). The overhead compared to a simple script is primarily development complexity, not computational cost — BT ticking runs at ~100 Hz with negligible CPU impact.

---

## Approach 3: learned navigation and exploration policies are lightweight but less proven

Neural navigation policies — particularly the GNM/ViNT/NoMaD family from UC Berkeley — offer a fundamentally different approach: replace hand-tuned planners with learned models that directly map sensor observations to actions or waypoints.

**NoMaD (ICRA 2024 Best Student Paper finalist)** is the most practical learned approach for this task. It combines exploration and goal-directed navigation in a single model through **goal masking**: when a goal image is provided, it navigates toward it; when the goal is masked, it explores autonomously using a diffusion-based action generator. NoMaD uses an EfficientNet-B0 + Transformer encoder (~30–60M parameters) with a lightweight action diffusion decoder. It runs on **NVIDIA Jetson Orin (~8 GB shared memory)**, making it trivially feasible on 8 GB discrete GPUs at an estimated **1–3 GB VRAM, 5–15 Hz inference**. The model achieved **25% improvement over ViNT** in undirected exploration and was deployed on LoCoBot, Jackal, drones, and quadrupeds without retraining. A ROS 2 port exists at `RobotecAI/visualnav-transformer-ros2`, providing nodes that publish predicted waypoints and convert them to velocity commands.

**The practical limitation:** NoMaD does not build an explicit map. It navigates by learned visual memory (a topological graph of visited locations) rather than an occupancy grid. For the DerpBot task — which requires building a map and detecting specific objects — NoMaD would need to be **paired with classical SLAM (slam_toolbox) and a separate detector (YOLO)**. This hybrid architecture works: NoMaD handles local navigation and exploration decisions, slam_toolbox builds the occupancy grid, and YOLO runs detection on the second GPU.

**ARiADNE and descendants** (ICRA 2023, RAL 2024) take a different approach: train attention-based DRL policies on 2D occupancy grid representations. The policy observes the partial map and selects exploration goals. The large-scale variant (`marmotlab/large-scale-DRL-exploration`) outperforms frontier-based methods by **11%** in trajectory efficiency and trains within **8 GB VRAM**. These policies operate on the SLAM-produced map rather than raw images, making sim-to-real transfer easier since LiDAR-based occupancy grids have a much smaller sim-to-real gap than camera images.

**VLFM (Vision-Language Frontier Maps, ICRA 2024)** bridges learned and classical approaches: it uses standard frontier detection on the occupancy map but scores frontiers using **BLIP-2 cosine similarity** between frontier views and the target object description. Deployed on Boston Dynamics Spot in a real office building, it achieved state-of-the-art zero-shot ObjectNav on HM3D, Gibson, and MP3D. The full VLFM stack (BLIP-2 + GroundingDINO + MobileSAM) required an RTX 4090 (16 GB), but a **lightweight variant using CLIP-ViT-B32 + YOLOv7** achieved better results with **2.3× less VRAM**, fitting within 8 GB.

**End-to-end approaches — a single network for navigation + exploration + detection — do not exist in deployable form as of 2025.** Vision-Language-Action models (RT-2, OpenVLA, π0, Octo) target manipulation, not exploration. NoMaD comes closest to end-to-end exploration but cannot detect specific objects. The consensus across recent literature is that **modular architectures with learned components outperform end-to-end systems** for exploration tasks.

**Sim-to-real transfer** has matured significantly. Salimpour et al. (January 2025) demonstrated RL policies trained in Isaac Sim achieving near-Nav2 performance on real ROS 2 robots. Key requirements: domain randomization of physics, rendering, and sensor noise; LSTM layers for temporal dependencies; and multi-faceted reward design. The sim-to-real gap is smallest for LiDAR-based policies, making DerpBot's 2D LiDAR an advantage.

| Learned model | Parameters | VRAM | Speed | Exploration? | Map building? |
|--------------|-----------|------|-------|-------------|--------------|
| NoMaD | ~30–60M | ~1–3 GB | 5–15 Hz | ✅ Native | ❌ Needs SLAM |
| GNM | ~10M | <0.5 GB | >100 Hz | ❌ Goal-only | ❌ Needs SLAM |
| ARiADNE | ~5–10M | <1 GB | 20–50 Hz | ✅ On map | ❌ Needs SLAM |
| VLFM (lightweight) | ~200M | ~4–6 GB | ~3–5 Hz | ✅ Semantic | ❌ Needs SLAM |
| Active Neural SLAM | ~5–20M | ~1 GB | 15–30 Hz | ✅ Learned | ⚠️ Learned mapper |

---

## Approach 4: semantic SLAM builds maps that understand what they contain

Semantic SLAM tightly couples object detection with spatial mapping, producing representations where objects are first-class entities rather than afterthoughts. This enables object-goal navigation, semantic queries ("where is the nearest fire extinguisher?"), and active perception that prioritizes exploration of semantically interesting areas.

**Hydra (MIT-SPARK Lab) is the flagship system**, now fully ported to ROS 2 Jazzy. It builds a hierarchical 3D scene graph in real-time with five layers: metric-semantic mesh → objects → places (navigable topological graph) → rooms → building. Each object node stores a semantic label, 3D centroid, bounding volume, and optionally CLIP embeddings for open-vocabulary queries. Hydra's geometric processing (ESDF construction, place extraction, room segmentation) runs on **CPU only**; GPU is used solely for the semantic segmentation frontend (FastSAM + CLIP). The GitHub repository (`MIT-SPARK/Hydra`, 802 stars) is actively maintained with the ROS 1 version archived in favor of ROS 2 as of July 2025. On the DerpBot's hardware, **FastSAM (~2–3 GB) + CLIP ViT-L/14 (~2 GB) fits comfortably on one GPU** while detection runs on the other.

**Clio (IEEE RA-L 2024)** extends Hydra with **task-driven open-set understanding**. Given natural language task descriptions ("find the brown textbook"), Clio uses Information Bottleneck theory to cluster 3D primitives at the right granularity — retaining objects relevant to the task while merging irrelevant detail. It runs on a **laptop carried by Boston Dynamics Spot** using FastSAM + CLIP, demonstrating real-time feasibility on edge hardware. Clio's key advantage for the DerpBot task: it dynamically adjusts scene graph resolution based on which objects the robot needs to find.

**Khronos** (RSS 2024), also from MIT-SPARK, adds temporal tracking to scene graphs, handling dynamic environments where objects move. It runs on ROS 2 Jazzy with Ubuntu 24.04 support.

**ConceptGraphs** (ICRA 2024, 727 GitHub stars) pioneered open-vocabulary 3D scene graphs using SAM + CLIP + GPT-4, but processes RGB-D sequences **offline** and requires ~12–16 GB VRAM. **HOV-SG** (RSS 2024) builds hierarchical floor→room→object graphs with open-vocabulary features but similarly lacks ROS integration and runs offline. Both represent important intellectual contributions but are not deployable on DerpBot without significant engineering.

**The practical detection-to-3D pipeline** for this approach uses the RGBD camera's depth channel directly. For each 2D detection, the system samples the median depth within the bounding box, back-projects to 3D using camera intrinsics (`P_cam = K⁻¹ · [u,v,1] · depth`), and transforms to the world frame via TF2. An object database maintains running average positions, CLIP embeddings, observation counts, and confidence scores. Objects are confirmed after **≥3 consistent observations** from different viewpoints, filtering false positives through spatial consistency (<0.3m position variance) and semantic consistency (CLIP cosine similarity >0.7 across views).

**Active perception** strategies guide exploration toward likely object locations. VLFM-style semantic frontier scoring uses VLM features to prioritize frontiers near semantically promising areas. **SEEK** (RSS 2024) distills LLM knowledge about object co-occurrences into a lightweight Relational Semantic Network that runs on CPU without internet — predicting that "towels are near sinks" to direct the robot toward bathrooms when searching for towels. The most advanced approach (arXiv 2510.05430, October 2025) uses LLMs to **sample plausible scene graphs of unobserved regions**, computing information gain for each potential waypoint.

**Feasibility on DerpBot:**

| Component | GPU | VRAM | Real-time? |
|-----------|-----|------|-----------|
| Hydra geometric core | CPU | — | ✅ |
| FastSAM (segmentation) | GPU 1 | ~2–3 GB | ✅ (~25 FPS) |
| CLIP ViT-L/14 (embeddings) | GPU 1 | ~2 GB | ✅ |
| YOLOE26-S (detection) | GPU 0 | ~1–2 GB | ✅ (50+ FPS) |
| Object database + tracking | CPU | — | ✅ |

---

## Approach 5: LLM/VLM orchestrators bring commonsense reasoning to exploration

The most architecturally novel approach uses large language models as high-level exploration planners that reason about indoor layouts, object-room associations, and exploration strategy in natural language — while classical systems handle low-level execution.

**ARNA (Stanford/NASA JPL, June 2025)** is the strongest demonstration of this paradigm. It embeds an LVLM agent as a dynamic orchestrator that autonomously defines and executes task-specific workflows by calling robotic modules as "tools." The LVLM accesses a tool library spanning perception (scene graph queries, image retrieval), reasoning (spatial reasoning, object search), and navigation (A* planning, frontier exploration). It maintains multimodal internal memory — a scene graph plus a sensor layer with images, depth maps, and occupancy grids. ARNA achieved state-of-the-art on the HM-EQA benchmark without handcrafted plans or fixed input representations. This represents the **exact paradigm needed for DerpBot**: the LLM reasons about which rooms to check, which frontiers look promising, and how to interpret mission descriptions, while Nav2 handles motion.

**L3MVN** provides a simpler but effective architecture: it builds a semantic map incrementally, detects objects at frontiers, then queries an LLM to score which frontier is most likely to contain the target. The combined score (geometric utility × LLM semantic likelihood) selects the exploration target. Zero-shot generalization works well — the LLM's commonsense about household layouts ("a TV is likely near a sofa") measurably improves exploration efficiency over pure frontier-based methods.

**Can useful LLMs run locally on 2× 8GB VRAM?** Yes. **Llama 3.1 8B at Q4_K_M quantization uses ~4.9 GB VRAM** and generates at 40+ tokens/second — fast enough for planning decisions every 5–10 seconds. Phi-3 Mini (3.8B, ~2.3 GB Q4) runs at 60+ tokens/second for even faster responses. For VLMs that can interpret camera images, **Phi-3-Vision (4.2B) runs in FP16 on an 8 GB RTX 4060 Ti** (~8.4 GB) or at Q4 quantization using ~3 GB. LLaVA-1.5-7B at 4-bit quantization fits under 8 GB. The practical GPU split is: **GPU 0 for LLM (~5 GB) + YOLO detection (~1.5 GB), GPU 1 for VLM (~4–5 GB)** with headroom for KV cache.

**The critical architectural principle is asynchronous operation.** The robot must never wait for the LLM. While navigating to the current frontier (10–30 seconds of transit), the LLM processes the next decision in the background. When the robot arrives, the LLM's recommendation is ready. Fallback logic reverts to nearest-frontier heuristic if the LLM times out. AESOP (RSS 2024) explicitly models LLM latency within MPC control design to maintain safety guarantees even with 1.5-second response delays.

**ROS 2 integration** options include `ros2_nanollm` (NVIDIA, ROS 2 native for Jetson/desktop), Ollama (simplest deployment: `ollama run llama3.1:8b`), and custom Python nodes using the pattern: subscribe to `/map`, `/detected_objects`, `/frontiers` → build text prompt → async query via `aiohttp` to local Ollama server → parse JSON response → publish `PoseStamped` goal to Nav2. Few-shot prompting (2–3 examples of exploration decisions) dramatically improves reliability for smaller models.

**Reliability concerns are manageable.** LLM hallucination is mitigated by always validating outputs against the actual frontier list — the LLM selects from enumerated options rather than generating arbitrary coordinates. Format failures are handled by constrained decoding or regex parsing with fallback to greedy frontier selection. Action history included in the prompt prevents oscillation loops. The BrainBody-LLM approach (2024) shows that closed-loop feedback (feeding environment errors back to the LLM) achieves **17% improvement** over open-loop methods.

---

## Open-vocabulary detectors fit easily within 8 GB VRAM

The open-vocabulary detection landscape has consolidated around two practical tiers for robotics: **YOLO-family models for real-time use and GroundingDINO for maximum accuracy**.

**YOLOE26 (Ultralytics, 2025) is the current best choice.** Built on the YOLO26 architecture with Re-parameterizable Region-Text Alignment, the open-vocabulary modules incur **zero inference cost** after re-parameterization. YOLOE26-S achieves **29.9% LVIS mAP at 161 FPS on a T4 GPU** — an 11.4-point improvement over YOLO-World-S. YOLOE26-L reaches 36.8% mAP with 32.3M parameters and 88.3B FLOPs. Both variants use ~1–3 GB VRAM during inference, leaving 5–7 GB free on each 8 GB GPU. If YOLOE26 is not yet available via Ultralytics pip, YOLO-World-L (46M params, 35.4% LVIS mAP, 52 FPS on V100, ~2–3 GB VRAM) is the proven fallback.

**GroundingDINO-T** (172M parameters, ~4–5 GB VRAM) delivers **52.5% COCO zero-shot AP** — roughly 50% higher accuracy than YOLO-World — but runs at only 5–8 FPS on an RTX 3080 Ti. This makes it unsuitable for real-time robotics but excellent for high-confidence periodic scans (e.g., when the robot stops at a frontier to do a detection sweep). GroundingDINO 1.5 Edge promises 75.2 FPS with TensorRT but remains API-only.

**OWLv2** (Google DeepMind) excels on rare categories (44.6% AP on LVIS rare classes) but runs at only 5–15 FPS. **Florence-2** is a versatile sequence-to-sequence model handling detection, captioning, and grounding but processes ~1 image/second — too slow for real-time. Both are better suited for offline analysis or one-shot scene understanding. **RAM++** produces image-level tags without bounding boxes and is useful only as a first-stage scene tagger feeding vocabulary into YOLO-World.

For **persistent tracking**, ByteTrack (IoU-only association, zero GPU overhead, CPU-based Kalman filter) is the pragmatic choice. BoT-SORT adds appearance features (~200 MB extra VRAM) for more robust ID maintenance across occlusions. Both are built into Ultralytics YOLO via config YAML. The **2D-to-3D projection** pipeline uses `depth_image_proc` for depth registration, `image_geometry.PinholeCameraModel` for back-projection, and TF2 for camera-to-world transform — all standard ROS 2 Jazzy packages.

---

## Revolutionary approaches that are promising but not yet practical

Several cutting-edge research directions could transform autonomous exploration within 3–5 years but remain infeasible for DerpBot's hardware and reliability requirements today.

**3D Gaussian Splatting SLAM** is the most tantalizing near-term prospect. GS-SLAM (CVPR 2024), SplaTAM, and RTG-SLAM (SIGGRAPH 2024) demonstrate dense 3D reconstruction from RGB-D streams using Gaussian primitives, with rendering at **30+ FPS** on modern GPUs. WildGS-SLAM (CVPR 2025) handles dynamic environments using DINOv2 uncertainty estimation. OpenGS-Fusion (IROS 2025) adds open-vocabulary language features to Gaussians for semantic understanding. The blocker is memory: Gaussian count grows with scene size, and online SLAM runs at only **3–15 FPS** while consuming 8–16+ GB VRAM. MemGS (IROS 2025) and Compact GS-SLAM address this with pruning and codebooks, but neither is production-ready. Once sub-1 GB GS-SLAM implementations emerge, they could replace both occupancy-grid SLAM and separate 3D reconstruction.

**Robot swarm exploration** with distributed SLAM has advanced considerably. Multi-SLAM (IEEE 2025) achieves globally optimized map merging across distributed robots, and LUOJIA Explorer (2025) provides a ROS 2-based multi-robot system using DDS communication. The fundamental challenges remain communication bandwidth constraints, inter-robot loop closure computation, and the lack of a robust turnkey package — each robot still needs its own full perception and planning stack.

**Foundation model world models** — UniSim (ICLR 2024 Outstanding Paper), GAIA-2 (Wayve), Dreamer 4 (September 2025) — enable planning by imagining future states. Dreamer 4 is the most practical, using a scalable block-causal transformer that achieves real-time interactive inference on a single GPU. DreamerNav applied DreamerV3 to real quadruped indoor navigation. However, training requires massive compute (48+ H100 GPUs for billion-parameter models), and pixel-based planning demands high-end GPUs for inference. NVIDIA Cosmos (2025) and Meta's V-JEPA 2 (2025) push toward general-purpose world models but remain far from edge deployment.

**Neuromorphic and event cameras** offer microsecond-latency perception with 120+ dB dynamic range — ideal for fast-moving robots in challenging lighting. Prophesee EVK4 HD provides ROS 2 drivers publishing `EventArray` messages. Science Robotics 2025 demonstrated ultra-energy-efficient neuromorphic localization. The barriers are hardware cost ($3,000–5,000 per sensor versus $200 for a RealSense), a tiny deep learning ecosystem, and no turnkey event-camera SLAM for ROS 2. These become practical when event cameras reach $500 and SNN training frameworks (Intel's lava-dl) mature.

**Diffusion policies for robot control** (RSS 2023, IJRR 2024) achieve 46.9% average improvement over prior methods across manipulation tasks. RDT-1B (ICLR 2025) demonstrated a 1.2B-parameter diffusion foundation model for bimanual manipulation. Smaller diffusion policies (~93M parameters) could fit in 8 GB VRAM, but all current systems target manipulation, not autonomous exploration. Adapting diffusion policies to generate exploration trajectories rather than manipulation actions is an open research problem with significant potential.

---

## Conclusion: matching architecture to mission requirements

The five approaches form a clear spectrum from proven to experimental. **The classical modular pipeline (Approach 1)** should be the baseline for any team — it works today, requires minimal tuning, and leaves both GPUs nearly idle. **Adding behavior tree orchestration (Approach 2)** costs only development time, not compute, and provides the structured mission management (time limits, parallel detection, recovery) that competition or deployment scenarios demand.

The three advanced approaches each add a distinct capability worth the complexity cost. **Learned exploration (Approach 3)** replaces hand-tuned frontier heuristics with policies that exploit structural regularities in indoor layouts — NoMaD at 1–3 GB VRAM is the most deployment-ready option. **Semantic SLAM (Approach 4)** via Hydra/Clio transforms the map from a geometry-only grid into a queryable scene graph — and its ROS 2 Jazzy port with FastSAM + CLIP fits within the GPU budget. **LLM orchestration (Approach 5)** brings commonsense reasoning about object-room associations — Llama 3.1 8B at Q4 quantization runs locally at 40+ tokens/second in ~5 GB VRAM, fast enough for asynchronous planning decisions during frontier transit.

The most promising hybrid for DerpBot combines elements across approaches: **slam_toolbox for mapping, Nav2 with BT orchestration for navigation, YOLOE26 for detection (~2 GB on GPU 0), and an 8B LLM or Phi-3-Vision VLM for semantic frontier scoring (~5 GB on GPU 1)**. This configuration uses roughly 7 GB per GPU, leaves headroom for inference peaks, and provides both the reliability of classical systems and the intelligence of foundation models. The detection-to-3D projection pipeline, persistent object tracking, and time-managed BT mission loop are independent of which exploration strategy sits on top — making the architecture easy to iterate on as these rapidly evolving approaches mature.