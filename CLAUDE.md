# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project: mira_pybullet_sim

A lightweight, headless, CPU-only PyBullet underwater AUV simulator for ROS2 that replicates the real robot's `/master` interface. Built for SAUVC competition tasks.

## Build & Run

```bash
# Build all packages
cd ~/DNT/mira_sim_nano
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# Run headless (default)
ros2 launch mira_pybullet_sim sim.launch.py

# Run with GUI (PyBullet visualizer)
ros2 launch mira_pybullet_sim sim.launch.py gui:=true

# Drive interactively (separate terminal)
ros2 run mira_pybullet_sim teleop_keyboard

# Run non-interactive test (each DOF in sequence)
ros2 run mira_pybullet_sim scripted_test

# Check publish rates
ros2 topic hz /master/telemetry   # should be ~100 Hz
ros2 topic hz /bluerov2/odometry   # should be ~50 Hz
```

## Architecture

### Packages
- **`src/custom_msgs/`**: ament_cmake package with two ROS2 message definitions:
  - `Commands.msg`: 6-DOF command input (forward, lateral, thrust, roll, pitch, yaw) + arm state.
  - `Telemetry.msg`: Vehicle state output (attitude, pressure, battery, thruster allocations).

- **`src/mira_pybullet_sim/`**: ament_python package with the simulator:
  - `physics.py`: Pure PyBullet world (pool, obstacles, AUV physics). No ROS deps; testable standalone.
  - `sim_node.py`: ROS2 node subscribing `/master/commands`, publishing `/master/telemetry` @100Hz + `/bluerov2/odometry` @50Hz.
  - `conventions.py`: Frame/attitude helpers (ENU↔NED, quaternion conversions, thruster allocation).
  - `teleop_keyboard.py` / `scripted_test.py`: Control input utilities.
  - `config/sim_params.yaml`: All tunable parameters (physics, pool, obstacles, spawn pose).
  - `launch/sim.launch.py`: Entry point with `gui` arg.

### Coordinate Frames
- **World (ENU):** +X east (right), +Y north (forward), +Z up. Water surface at z=0. Pool runs 25m (+Y) × 16m (+X).
- **Body (REP-103):** +X forward, +Y left, +Z up (vehicle-local).
- **Telemetry attitude:** Reported in NED-style aerospace convention (roll/pitch/yaw); conversion from ENU handled in `conventions.py`.

### Physics Model
- Single rigid cuboid AUV, slightly positive buoyancy, self-righting via center-of-buoyancy offset.
- Command mapping: PWM [1100, 1900] → norm [-1, 1] (1500 neutral) → per-axis wrench (linear force + torque).
- Per-axis hydrodynamic drag (linear + quadratic terms).
- Static obstacles: pool (with light blue visual), starting ring (flat gate), inverted-U gate, 4 drums, 3 flares (poles with free-moving golf balls), water surface plane (visual only).

### Config-Driven Design
**All magic numbers live in `config/sim_params.yaml`.** Changing pool dims, obstacle positions, AUV spawn, or physics parameters requires no code edits—just edit YAML and relaunch. The launch file loads the YAML into ROS2 parameters, which are read at node startup.

## Key Files

| File | Purpose |
|------|---------|
| `src/mira_pybullet_sim/physics.py` | PyBullet world: bodies, forces, collisions. Callable standalone for testing. |
| `src/mira_pybullet_sim/sim_node.py` | ROS2 glue: subscribes commands, publishes telemetry/odometry. |
| `src/mira_pybullet_sim/conventions.py` | Attitude/frame math (Euler↔quaternion, ENU↔NED, thruster PWM allocation). |
| `config/sim_params.yaml` | All tunable parameters: physics, pool, AUV spawn, obstacles (ring, gate, drums, flares). |
| `launch/sim.launch.py` | Launch entry point with `gui` arg; loads YAML params. |

## SAUVC Course Layout

The course is a 25m (Y) × 16m (X) pool with:
- **Starting zone** (y ∈ [-12.5, -8.5]): 1.4m × 1.4m ring gate at y≈-11.8; AUV spawns nearby.
- **Orange-flare zone** (y ∈ [-8.5, -4.5]): 1 orange post (visual marker).
- **R/B/Y-flare zone** (y ∈ [-4.5, +3.5]): 3 colored flares (red/blue/yellow poles with free-moving golf balls).
- **Gate line** (y≈+3.5): Inverted-U gate for the AUV to pass through.
- **Target zone** (y ∈ [+10.5, +12.5]): 4 drums (1 blue, 3 red) to navigate.

All positions are configurable via `config/sim_params.yaml`.

## Common Tasks

- **Adjust AUV physics (mass, drag, buoyancy):** Edit `config/sim_params.yaml` under the body/drag/buoyancy sections.
- **Move an obstacle:** Edit `*_pos` or `*_positions` in `config/sim_params.yaml` (e.g., `gate_pos`, `drum_positions`).
- **Change AUV spawn location/heading:** Edit `start_x`, `start_y`, `start_z`, `start_yaw`.
- **Disable an obstacle:** Set `*_enabled: false` in YAML (e.g., `ring_enabled: false`).
- **Debug physics without ROS:** `python3 -m mira_pybullet_sim.physics` (headless smoke test).
- **Verify message rates:** `ros2 topic hz /master/telemetry` (expect ~100 Hz).

## Notes

- **Reference sim:** The old Stonefish sim at `~/DNT/mira_sim/` is read-only reference only. This PyBullet sim is a separate, standalone project.
- **No SITL/MAVLink:** This sim *is* the vehicle at the `/master` interface; there is no Pixhawk, ArduSub, or MAVLink bridge.
- **Headless by default:** PyBullet DIRECT mode (no GPU needed). GUI mode available for local debug only.
- **ROS2 Jazzy, Python 3.12, colcon build system.**
- **pybullet 3.2.7** installed via pip (system interpreter, not venv).
