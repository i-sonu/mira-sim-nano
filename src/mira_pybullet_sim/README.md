# mira_pybullet_sim

A lightweight, **CPU-only / headless** PyBullet underwater-AUV simulator that speaks
the team's **real-robot ROS2 `/master` interface**. Control code written once runs
identically against this sim and the real vehicle.

On the real robot a "master" node bridges MAVLink ↔ ROS. **This sim replaces that
node** — it *is* the vehicle at the `/master` level. There is **no SITL / ArduSub /
MAVLink / Pixhawk** in the loop, and **no cameras**. This is a deliberate Phase-1
decision. It is a separate project from the Stonefish sim (`~/DNT/mira_sim/`); that
sim is untouched.

## Phase 1 scope

A pool (hollow box of "water") containing a single rigid **cuboid** AUV that:
- floats (slightly positive buoyancy) and self-rights in roll/pitch,
- moves in all 6 DOF (surge, sway, heave, roll, pitch, yaw) from `/master/commands`,
- reports state on `/master/telemetry` (+ a debug `/bluerov2/odometry`).

Forces are a **single whole-body wrench** — no per-thruster geometry/physics. The
8 `thruster_pwms` in telemetry are **cosmetic only**.

---

## Build & run

### Dependencies
- ROS 2 Jazzy, Python 3.12, `colcon`.
- PyBullet (pip): `pip install pybullet`
- `custom_msgs` (built from this workspace).

### Build
```bash
cd ~/DNT/mira_sim_nano
colcon build
source install/setup.bash
```

### Run headless (default, no GPU / no display)
```bash
ros2 launch mira_pybullet_sim sim.launch.py
```

### Run with the PyBullet GUI (local debug only — needs a display)
```bash
ros2 launch mira_pybullet_sim sim.launch.py gui:=true
```

### Drive it
Interactive keyboard (own terminal, real TTY):
```bash
ros2 run mira_pybullet_sim teleop_keyboard
```
Non-interactive scripted sequence (exercises each DOF in turn):
```bash
ros2 run mira_pybullet_sim scripted_test
```

### Standalone physics smoke test (no ROS, no display)
```bash
python3 -m mira_pybullet_sim.physics
```

---

## ROS 2 interface

| Direction | Topic | Type | Rate |
|-----------|-------|------|------|
| Subscribe | `/master/commands` | `custom_msgs/Commands` | (last-write-wins) |
| Publish | `/master/telemetry` | `custom_msgs/Telemetry` | 100 Hz |
| Publish | `/bluerov2/odometry` | `nav_msgs/Odometry` | 50 Hz (debug/rviz) |

**`/master/telemetry` is the control interface.** `/bluerov2/odometry` is clean
ENU/REP-103 ground truth for rviz/debug only — control should read `/master/telemetry`.

`imu_ned` is **not** published. No `sim_master` bridge runs — this node is the master.

---

## Conventions (read carefully)

### Frames
- **World (PyBullet):** ENU — +X east, +Y north, **+Z up**. Water surface at `z = 0`.
- **Body (PyBullet / odometry):** REP-103 — **+X forward, +Y left, +Z up**.
- **Depth:** `depth = max(0, water_surface_z - z)` — positive below the surface.

### Command → wrench mapping (`/master/commands`)
Each channel: `norm = clamp((pwm - 1500) / 400, -1, 1)`. PWM is 1100–1900 µs, 1500
neutral. `1.0` maps **linearly** to that axis's max force/torque (no thrust-curve
saturation). `mode` is **ignored** in Phase 1 (treated as direct/MANUAL).
`servo1`/`servo2` are ignored.

| Command | DOF | Body action | Max param | Sign |
|---------|-----|-------------|-----------|------|
| `forward` | surge | force +X (body) | `max_surge_n` | pwm>1500 → forward |
| `lateral` | sway | force +Y (body) | `max_sway_n` | **pwm>1500 → left (+Y)** |
| `thrust` | heave | force +Z (body) | `max_heave_n` | pwm>1500 → up |
| `roll` | roll | torque about +X | `max_roll_nm` | right-hand rule about +X |
| `pitch` | pitch | torque about +Y | `max_pitch_nm` | right-hand rule about +Y |
| `yaw` | yaw | torque about +Z | `max_yaw_nm` | right-hand rule about +Z |

The lateral sign is applied **once** (positive `lateral` → +Y body = left). This
deliberately does **not** reproduce the old `sim_master` lateral double-negation.

**`arm == False` → zero wrench**, regardless of command values. The latest command is
applied each physics step (last-write-wins; multiple publishers are not blended).

### Telemetry attitude convention (`roll/pitch/yaw`, `q1..q4`)
Reported in a **MAVLink ATTITUDE / NED-style** aerospace convention (+X forward,
+Y right, +Z down, yaw about world-down), converted from the internal ENU/REP-103
attitude by flipping the Y and Z body axes:
```
roll_ned  =  roll_enu
pitch_ned = -pitch_enu
yaw_ned   = -yaw_enu        (wrapped to (-pi, pi])
rollspeed =  wx_body ; pitchspeed = -wy_body ; yawspeed = -wz_body
```
`q1..q4` = `(w, x, y, z)` built from the NED Euler angles, so the quaternion and
Euler triplet are mutually consistent. `heading` = `round(deg(yaw_ned)) mod 360`,
in `[0, 359]`. **These signs are self-consistent but provisional** and will be pinned
against the real Pixhawk in a later phase (see `conventions.py`).

`/bluerov2/odometry` uses the **clean** ENU/REP-103 attitude (no NED flip); its twist
is expressed in the body (`child_frame_id`) frame per REP-103.

### Pressure
```
external_pressure = 101325 + water_density * gravity * depth      [Pa, absolute]
```
With defaults (`rho=1000`, `g=9.81`) this is exactly invertible by control code:
`depth = (external_pressure - 101325) / (1000 * 9.81)`.
`internal_pressure` is a constant `101325.0` (faked).

### Cosmetic thruster PWMs
`thruster_pwms[8]` is computed by running the documented 8-thruster allocation on the
normalized command and mapping to PWM — **for telemetry realism only; it never affects
physics**:
```
T0=surge+yaw+sway   T1=surge-yaw-sway   T2=-surge-yaw+sway   T3=-surge+yaw-sway
T4=heave+pitch+roll T5=heave+pitch-roll T6=heave-pitch+roll  T7=heave-pitch-roll
clamp each to [-1,1];  pwm_i = 1500 + Ti*400
```

### Telemetry: ground-truth vs faked
| Field | Source |
|-------|--------|
| `timestamp` | **ground truth** (sim time, s) |
| `arm` | **ground truth** (last commanded arm) |
| `roll/pitch/yaw`, `q1..q4` | **ground truth** orientation (NED-style convention) |
| `rollspeed/pitchspeed/yawspeed` | **ground truth** body angular velocity |
| `heading` | **ground truth** (derived from yaw) |
| `external_pressure` | **ground truth** (from depth) |
| `battery_voltage` | faked constant `16.0` |
| `internal_pressure` | faked constant `101325.0` |
| `thruster_pwms[8]` | cosmetic (from command allocation; not physics) |
| `imu_gyro_*`, `imu_*acc`, `imu_gyro_compass_*` | **unimplemented**, set to `0` |

---

## Configuration (`config/sim_params.yaml`)

All magic numbers are parameters. Defaults are BlueROV2-scale and tunable.

| Param | Default | Meaning |
|-------|---------|---------|
| `gui` | `false` | PyBullet GUI (local debug; needs display) |
| `telemetry_rate` | `100.0` | `/master/telemetry` Hz |
| `odom_rate` | `50.0` | `/bluerov2/odometry` Hz |
| `timestep` | `1/240` | fixed physics step (s) |
| `mass` | `11.0` | AUV mass (kg) |
| `half_extents` | `[0.30,0.20,0.20]` | box half-dims x,y,z (m) = 0.60 x 0.40 x 0.40 |
| `start_z` | `-0.5` | initial spawn depth (m) |
| `gravity` | `9.81` | m/s² |
| `water_density` | `1000.0` | kg/m³ (pressure + buoyancy) |
| `water_surface_z` | `0.0` | surface plane (m) |
| `displaced_volume` | `0.01155` | buoyant volume (m³); tuned slightly positive |
| `buoyancy_ramp_height` | `0.05` | submerged-fraction ramp band at surface (m) |
| `cob_offset_body` | `[0,0,0.02]` | center of buoyancy above CoG (m) → self-rights |
| `drag_linear` | `[40,60,80]` | linear drag N/(m/s) per surge/sway/heave |
| `drag_quadratic` | `[60,90,120]` | quadratic drag N/(m/s)² |
| `angular_drag_linear` | `[4,4,3]` | angular linear drag Nm/(rad/s) roll/pitch/yaw |
| `angular_drag_quadratic` | `[6,6,4]` | angular quadratic drag Nm/(rad/s)² |
| `max_surge_n` | `100.0` | max surge force (N) |
| `max_sway_n` | `80.0` | max sway force (N) |
| `max_heave_n` | `100.0` | max heave force (N) |
| `max_roll_nm` | `5.0` | max roll torque (Nm) |
| `max_pitch_nm` | `5.0` | max pitch torque (Nm) |
| `max_yaw_nm` | `8.0` | max yaw torque (Nm) |
| `pool_length`/`pool_width`/`pool_depth` | `25/16/2` | pool dims (m) |
| `pool_wall_thickness` | `0.2` | wall thickness (m) |
| `surface_plane_enabled` | `true` | show the water surface visual plane |
| `surface_plane_thickness` | `0.0001` | plane thickness (m, 0.1 mm) |
| `ring_enabled` | `true` | build the static square gate/ring |
| `ring_size` | `1.40` | ring outer side length (m) |
| `ring_thickness` | `0.02` | ring bar cross-section (m, square) |
| `ring_pos` | `[0,3,-1]` | ring center in world (m); +y forward, -z deeper |
| `ring_rpy` | `[0,0,0]` | ring orientation (rad); `[1.5708,0,0]` lies flat |
| `post_enabled` | `true` | build the static orange post |
| `post_height` | `1.60` | post height (m, along local Z) |
| `post_thickness` | `0.15` | post cross-section (m, square) |
| `post_pos` | `[2,0,-1]` | post center in world (m); +y forward, -z deeper |
| `post_rpy` | `[0,0,0]` | post orientation (rad); `[1.5708,0,0]` lies horizontal |
| `gate_enabled` | `true` | build the static inverted-U gate |
| `gate_width` | `1.50` | top-bar outer span, leg to leg (m) |
| `gate_height` | `1.00` | leg height / opening height (m) |
| `gate_thickness` | `0.15` | gate bar cross-section (m, square) |
| `gate_pos` | `[-3,0,-2]` | gate base center on floor (m); set z = -pool_depth |
| `gate_rpy` | `[0,0,0]` | gate orientation (rad); opening faces ±Y |
| `drums_enabled` | `true` | build the static drums |
| `drum_diameter` | `0.60` | drum diameter (m) |
| `drum_depth` | `0.30` | drum height (m) |
| `drum_positions` | `[…]` | flat `x,y,z` triples, one per drum |
| `drum_colors` | `[blue,red,red,red]` | color name per drum by index |
| `flares_enabled` | `true` | build the flares + free golf balls |
| `flare_height` | `0.80` | pole height (m) |
| `flare_pole_diameter`/`flare_base_diameter`/`flare_base_height` | `0.03/0.12/0.03` | pole + foot dims (m) |
| `flare_ball_diameter`/`flare_ball_mass` | `0.043/0.046` | golf ball size (m) / mass (kg) |
| `flare_positions` | `[…]` | flat `x,y,z` triples, one per flare (base on floor) |
| `flare_colors` | `[red,yellow,blue]` | pole color per flare by index |

---

## Quick verification

```bash
# terminal 1
ros2 launch mira_pybullet_sim sim.launch.py

# terminal 2
ros2 topic hz /master/telemetry          # ~100 Hz
ros2 topic hz /bluerov2/odometry         # ~50 Hz
ros2 run mira_pybullet_sim scripted_test # watch each DOF move in turn
ros2 topic echo /master/telemetry        # check external_pressure vs depth
```
- `arm=True, forward=1900` → odometry `position.x` increases; other channels each move
  only their own DOF (modulo passive buoyant righting).
- `arm=False` → no motion regardless of commands.
- Neutral → cuboid rises to near the surface and stays; perturbing roll/pitch → it
  self-rights.
