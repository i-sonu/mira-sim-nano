# Commands

A running list of commands for the mira_pybullet_sim workspace.

## Start the simulator (visual / GUI)

```bash
cd ~/DNT/mira_sim_nano
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mira_pybullet_sim sim.launch.py gui:=true
```

This opens the PyBullet 3D window with the white pool and the AUV cuboid.

To drive it, open a **second terminal** and run one of:

```bash
cd ~/DNT/mira_sim_nano
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mira_pybullet_sim teleop_keyboard   # WASD/QE/RF to move, SPACE to arm
# or
ros2 run mira_pybullet_sim scripted_test     # auto-runs through each DOF
```
