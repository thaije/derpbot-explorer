# derpbot-explorer
Various robot autonomy implementations for achieving highscores on the Autonomous Robotics Simulation Testbed (ARST).

Full plan for first simple autonnomy: [`docs/simple_autonomy.md`](docs/simple_autonomy.md)   
Roadmap to more complex autonomy: [`docs/five approaches to robot autonomy.md`](docs/five approaches to robot autonomy.md)  
Agent state: [`docs/AGENT_HANDOFF.md`](docs/AGENT_HANDOFF.md)



## Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (`ros-jazzy-ros-gz*`)
- `robot_state_publisher`, `ros_gz_bridge`, `ros_gz_sim`, `teleop_twist_keyboard`
- **Python 3.12** — `rclpy` is compiled for 3.12; `python3` may resolve to a different interpreter
- **uv** — Python package/tool runner (`pip install uv` or see [docs.astral.sh/uv](https://docs.astral.sh/uv)): `uv venv` > source venv > `uv pip install -r requirements.txt`
- **ast-grep** — structural code search (`npm install -g @ast-grep/cli`)
- **Serena MCP** — symbol-level code navigation for AI agents (auto-installed via uv/uvx; see `~/.claude.json`)