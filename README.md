# mira-sim-nano

A lightweight, headless, CPU-only PyBullet underwater AUV simulator for ROS2. Built for roboticists without a GPU who want to test their ROS nodes against a simulated vehicle. Replicates the real robot's `/master` interface.

## Quick Start

```bash
cd ~/DNT/mira_sim_nano
source /opt/ros/jazzy/setup.bash
colcon build
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
