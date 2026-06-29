# mira-sim-nano

A lightweight, headless, CPU-only PyBullet underwater AUV simulator for ROS2. Built for roboticists without a GPU who want to test their ROS nodes against a simulated vehicle. Replicates the real robot's `/master` interface.

<img width="1849" height="1160" alt="Screenshot from 2026-06-25 04-14-29" src="https://github.com/user-attachments/assets/a936ace3-66c6-4893-9bbb-8debf2bba6ab" />
<img width="1849" height="1160" alt="Screenshot from 2026-06-25 04-15-31" src="https://github.com/user-attachments/assets/58f02f6a-d654-4eb4-acde-df5c18ee0cd9" />
<img width="1015" height="646" alt="Screenshot from 2026-06-25 04-15-54" src="https://github.com/user-attachments/assets/61a9a316-4e25-4609-ba02-16740b25be3c" />

## First-Time Setup (fresh laptop)

Assumes ROS 2 Jazzy is already installed. PyBullet is the only extra dependency.

```bash
# 1. Install PyBullet (the --break-system-packages --user flags are needed on Ubuntu 24.04)
pip install --break-system-packages --user pybullet

# 2. Clone the repo
git clone https://github.com/i-sonu/mira-sim-nano.git ~/DNT/mira_sim_nano
cd ~/DNT/mira_sim_nano

# 3. Source ROS *before* building (colcon builds the custom_msgs interfaces and needs the rosidl tooling)
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

> **Every new terminal** needs `source /opt/ros/jazzy/setup.bash` and `source install/setup.bash` before `ros2 run`/`ros2 launch` can find the package. Add them to `~/.bashrc` to avoid retyping. There is no venv/uv—PyBullet is the only pip dependency, and ROS's `rclpy` comes from the system Python.

## Quick Start

```bash
cd ~/DNT/mira_sim_nano
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run headless (default)
ros2 launch mira_pybullet_sim sim.launch.py

# Run with GUI visualization
ros2 launch mira_pybullet_sim sim.launch.py gui:=true
```

## Drive the AUV

In a separate terminal:
```bash
# Interactive keyboard control
ros2 run mira_pybullet_sim teleop_keyboard

# Or run a scripted test sequence
ros2 run mira_pybullet_sim scripted_test
```

## Topics

- **Subscribe:** `/master/commands` (custom_msgs/Commands) — control input
- **Publish:** `/master/telemetry` (custom_msgs/Telemetry) @ 100 Hz — vehicle state
- **Publish:** `/bluerov2/odometry` (nav_msgs/Odometry) @ 50 Hz — ground truth (ENU/REP-103)

## Configuration

All pool dimensions, obstacle positions, physics parameters, and AUV spawn location are tunable via `src/mira_pybullet_sim/config/sim_params.yaml`—edit YAML and relaunch, no code changes needed.

## SAUVC Course

The simulator includes a SAUVC competition pool layout with obstacle zones, gates, drums, and flares. See `CLAUDE.md` for details.

## Dependencies

- ROS 2 Jazzy, Python 3.12, colcon
- PyBullet 3.2.7 (installed via pip)
