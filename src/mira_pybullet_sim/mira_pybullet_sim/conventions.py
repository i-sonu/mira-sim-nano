"""Frame / attitude conventions and the cosmetic thruster allocation.

This module is the single source of truth for how the simulator converts between
the internal PyBullet world (ENU, +Z up, body = REP-103 x-forward/y-left/z-up) and
the values reported on the real-robot /master/telemetry interface (MAVLink-style).

Keep this isolated and self-consistent; exact signs vs. the real Pixhawk are pinned
in a later phase. See README "Conventions" section.
"""

import math


# --- Internal frames -------------------------------------------------------
#
# World (PyBullet): ENU. +X east, +Y north, +Z up. Water surface at z = 0.
# Body (PyBullet):  REP-103. +X forward, +Y left, +Z up.
#
# This is the clean convention used for /bluerov2/odometry (debug/rviz).


def quaternion_to_euler_zyx(qx, qy, qz, qw):
    """PyBullet quaternion (x, y, z, w) -> intrinsic ZYX (yaw, pitch, roll) in radians.

    Returns (roll, pitch, yaw) about body X, Y, Z respectively in the ENU/REP-103
    body frame. Standard aerospace ZYX decomposition.
    """
    # roll (X)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (Y)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # yaw (Z)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# --- Telemetry attitude convention (MAVLink ATTITUDE / NED-style) ----------
#
# The real /master/telemetry mirrors the Pixhawk ATTITUDE message, which uses an
# aerospace/NED body frame: +X forward, +Y right, +Z down, yaw about world-down.
#
# Our internal body frame is REP-103 (x-forward, y-LEFT, z-UP). Converting the
# clean ENU/REP-103 attitude to the NED-style ATTITUDE convention is a flip of the
# Y and Z body axes, which negates pitch and yaw (roll about the common forward
# axis is unchanged):
#
#     roll_ned  =  roll_enu
#     pitch_ned = -pitch_enu
#     yaw_ned   = -yaw_enu        (then wrapped to (-pi, pi])
#
# Body angular rates transform the same way (p, q, r about X, Y, Z):
#     rollspeed  =  wx_body
#     pitchspeed = -wy_body
#     yawspeed   = -wz_body
#
# NOTE: signs here are self-consistent but provisional; they will be pinned
# against the real Pixhawk in a later phase. Documented in the README.


def enu_attitude_to_ned(roll_enu, pitch_enu, yaw_enu):
    """(roll, pitch, yaw) REP-103/ENU -> NED-style ATTITUDE Euler angles (rad)."""
    roll_ned = roll_enu
    pitch_ned = -pitch_enu
    yaw_ned = wrap_pi(-yaw_enu)
    return roll_ned, pitch_ned, yaw_ned


def enu_rates_to_ned(wx, wy, wz):
    """Body angular velocity REP-103 -> NED-style (rollspeed, pitchspeed, yawspeed)."""
    return wx, -wy, -wz


def euler_to_quaternion_zyx(roll, pitch, yaw):
    """(roll, pitch, yaw) intrinsic ZYX -> quaternion (w, x, y, z).

    Used to fill telemetry q1..q4 from the NED-style Euler angles so that the
    reported quaternion and Euler triplet are mutually consistent.
    """
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


def heading_from_yaw_ned(yaw_ned):
    """NED yaw (rad) -> integer compass heading in [0, 359] degrees."""
    deg = math.degrees(yaw_ned)
    return int(round(deg)) % 360


def wrap_pi(angle):
    """Wrap an angle to (-pi, pi]."""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


# --- Cosmetic 8-thruster allocation ---------------------------------------
#
# COSMETIC ONLY: telemetry realism. This never touches physics. Inputs are the
# normalized [-1, 1] 6-DOF commands; outputs are 8 PWM values in microseconds.


def allocate_thruster_pwms(surge, sway, heave, roll, pitch, yaw):
    """Normalized 6-DOF command -> list of 8 thruster PWMs (microseconds, float).

    Allocation (matches the real stack's mixing signs):
        T0 = surge + yaw + sway     (front-right horizontal)
        T1 = surge - yaw - sway     (front-left  horizontal)
        T2 = -surge - yaw + sway    (back-right  horizontal)
        T3 = -surge + yaw - sway    (back-left   horizontal)
        T4 = heave + pitch + roll   (front-right vertical)
        T5 = heave + pitch - roll   (front-left  vertical)
        T6 = heave - pitch + roll   (back-right  vertical)
        T7 = heave - pitch - roll   (back-left   vertical)
    Each Ti is clamped to [-1, 1], then pwm_i = 1500 + Ti * 400.
    """
    t = [
        surge + yaw + sway,
        surge - yaw - sway,
        -surge - yaw + sway,
        -surge + yaw - sway,
        heave + pitch + roll,
        heave + pitch - roll,
        heave - pitch + roll,
        heave - pitch - roll,
    ]
    return [1500.0 + max(-1.0, min(1.0, ti)) * 400.0 for ti in t]
