"""ROS2 node: PyBullet AUV simulator at the real-robot /master interface.

This node REPLACES the real robot's MAVLink<->ROS master node. It IS the vehicle:
  Subscribes : /master/commands   (custom_msgs/Commands)
  Publishes  : /master/telemetry  (custom_msgs/Telemetry)   @ 100 Hz
  Publishes  : /bluerov2/odometry (nav_msgs/Odometry)       @  50 Hz  (debug/rviz)

No SITL / MAVLink / cameras. See README for conventions.
"""

import rclpy
from rclpy.node import Node

from custom_msgs.msg import Commands, Telemetry
from nav_msgs.msg import Odometry

from .physics import AuvSim, AuvConfig, Command
from . import conventions as cv


def _clamp_norm(pwm):
    """PWM (microseconds) -> normalized [-1, 1] about the 1500 neutral."""
    n = (float(pwm) - 1500.0) / 400.0
    return max(-1.0, min(1.0, n))


class SimNode(Node):
    def __init__(self):
        super().__init__("mira_pybullet_sim")

        self._declare_params()
        cfg = self._build_config()
        gui = self.get_parameter("gui").value

        self.sim = AuvSim(cfg, gui=gui)
        self.cmd = Command()  # disarmed, neutral
        self.cfg = cfg

        self.telemetry_rate = float(self.get_parameter("telemetry_rate").value)
        self.odom_rate = float(self.get_parameter("odom_rate").value)

        self.sub = self.create_subscription(
            Commands, "/master/commands", self._on_commands, 10)
        self.telem_pub = self.create_publisher(Telemetry, "/master/telemetry", 10)
        self.odom_pub = self.create_publisher(Odometry, "/bluerov2/odometry", 10)

        # Physics steps at the fixed sim timestep; telemetry/odom on their own timers.
        self.create_timer(cfg.timestep, self._on_physics)
        self.create_timer(1.0 / self.telemetry_rate, self._on_telemetry)
        self.create_timer(1.0 / self.odom_rate, self._on_odometry)

        self.get_logger().info(
            f"mira_pybullet_sim up (gui={gui}, dt={cfg.timestep:.5f}s, "
            f"telemetry={self.telemetry_rate:.0f}Hz, odom={self.odom_rate:.0f}Hz)")

    # -- params -------------------------------------------------------------
    def _declare_params(self):
        d = AuvConfig()
        self.declare_parameter("gui", False)
        self.declare_parameter("telemetry_rate", 100.0)
        self.declare_parameter("odom_rate", 50.0)
        self.declare_parameter("mass", d.mass)
        self.declare_parameter("half_extents", d.half_extents)
        self.declare_parameter("gravity", d.gravity)
        self.declare_parameter("water_density", d.water_density)
        self.declare_parameter("water_surface_z", d.water_surface_z)
        self.declare_parameter("displaced_volume", d.displaced_volume)
        self.declare_parameter("buoyancy_ramp_height", d.buoyancy_ramp_height)
        self.declare_parameter("cob_offset_body", d.cob_offset_body)
        self.declare_parameter("drag_linear", d.drag_linear)
        self.declare_parameter("drag_quadratic", d.drag_quadratic)
        self.declare_parameter("angular_drag_linear", d.angular_drag_linear)
        self.declare_parameter("angular_drag_quadratic", d.angular_drag_quadratic)
        self.declare_parameter("max_surge_n", d.max_surge_n)
        self.declare_parameter("max_sway_n", d.max_sway_n)
        self.declare_parameter("max_heave_n", d.max_heave_n)
        self.declare_parameter("max_roll_nm", d.max_roll_nm)
        self.declare_parameter("max_pitch_nm", d.max_pitch_nm)
        self.declare_parameter("max_yaw_nm", d.max_yaw_nm)
        self.declare_parameter("pool_length", d.pool_length)
        self.declare_parameter("pool_width", d.pool_width)
        self.declare_parameter("pool_depth", d.pool_depth)
        self.declare_parameter("pool_wall_thickness", d.pool_wall_thickness)
        self.declare_parameter("surface_plane_enabled", d.surface_plane_enabled)
        self.declare_parameter("surface_plane_thickness", d.surface_plane_thickness)
        self.declare_parameter("ring_enabled", d.ring_enabled)
        self.declare_parameter("ring_size", d.ring_size)
        self.declare_parameter("ring_thickness", d.ring_thickness)
        self.declare_parameter("ring_pos", d.ring_pos)
        self.declare_parameter("ring_rpy", d.ring_rpy)
        self.declare_parameter("post_enabled", d.post_enabled)
        self.declare_parameter("post_height", d.post_height)
        self.declare_parameter("post_thickness", d.post_thickness)
        self.declare_parameter("post_pos", d.post_pos)
        self.declare_parameter("post_rpy", d.post_rpy)
        self.declare_parameter("gate_enabled", d.gate_enabled)
        self.declare_parameter("gate_width", d.gate_width)
        self.declare_parameter("gate_height", d.gate_height)
        self.declare_parameter("gate_thickness", d.gate_thickness)
        self.declare_parameter("gate_pos", d.gate_pos)
        self.declare_parameter("gate_rpy", d.gate_rpy)
        self.declare_parameter("drums_enabled", d.drums_enabled)
        self.declare_parameter("drum_diameter", d.drum_diameter)
        self.declare_parameter("drum_depth", d.drum_depth)
        self.declare_parameter("drum_positions", d.drum_positions)
        self.declare_parameter("drum_colors", d.drum_colors)
        self.declare_parameter("flares_enabled", d.flares_enabled)
        self.declare_parameter("flare_height", d.flare_height)
        self.declare_parameter("flare_pole_diameter", d.flare_pole_diameter)
        self.declare_parameter("flare_base_diameter", d.flare_base_diameter)
        self.declare_parameter("flare_base_height", d.flare_base_height)
        self.declare_parameter("flare_ball_diameter", d.flare_ball_diameter)
        self.declare_parameter("flare_ball_mass", d.flare_ball_mass)
        self.declare_parameter("flare_positions", d.flare_positions)
        self.declare_parameter("flare_colors", d.flare_colors)
        self.declare_parameter("timestep", d.timestep)
        self.declare_parameter("start_x", d.start_x)
        self.declare_parameter("start_y", d.start_y)
        self.declare_parameter("start_z", d.start_z)
        self.declare_parameter("start_yaw", d.start_yaw)

    def _build_config(self):
        g = self.get_parameter
        return AuvConfig(
            mass=float(g("mass").value),
            half_extents=[float(x) for x in g("half_extents").value],
            gravity=float(g("gravity").value),
            water_density=float(g("water_density").value),
            water_surface_z=float(g("water_surface_z").value),
            displaced_volume=float(g("displaced_volume").value),
            buoyancy_ramp_height=float(g("buoyancy_ramp_height").value),
            cob_offset_body=[float(x) for x in g("cob_offset_body").value],
            drag_linear=[float(x) for x in g("drag_linear").value],
            drag_quadratic=[float(x) for x in g("drag_quadratic").value],
            angular_drag_linear=[float(x) for x in g("angular_drag_linear").value],
            angular_drag_quadratic=[float(x) for x in g("angular_drag_quadratic").value],
            max_surge_n=float(g("max_surge_n").value),
            max_sway_n=float(g("max_sway_n").value),
            max_heave_n=float(g("max_heave_n").value),
            max_roll_nm=float(g("max_roll_nm").value),
            max_pitch_nm=float(g("max_pitch_nm").value),
            max_yaw_nm=float(g("max_yaw_nm").value),
            pool_length=float(g("pool_length").value),
            pool_width=float(g("pool_width").value),
            pool_depth=float(g("pool_depth").value),
            pool_wall_thickness=float(g("pool_wall_thickness").value),
            surface_plane_enabled=bool(g("surface_plane_enabled").value),
            surface_plane_thickness=float(g("surface_plane_thickness").value),
            ring_enabled=bool(g("ring_enabled").value),
            ring_size=float(g("ring_size").value),
            ring_thickness=float(g("ring_thickness").value),
            ring_pos=[float(x) for x in g("ring_pos").value],
            ring_rpy=[float(x) for x in g("ring_rpy").value],
            post_enabled=bool(g("post_enabled").value),
            post_height=float(g("post_height").value),
            post_thickness=float(g("post_thickness").value),
            post_pos=[float(x) for x in g("post_pos").value],
            post_rpy=[float(x) for x in g("post_rpy").value],
            gate_enabled=bool(g("gate_enabled").value),
            gate_width=float(g("gate_width").value),
            gate_height=float(g("gate_height").value),
            gate_thickness=float(g("gate_thickness").value),
            gate_pos=[float(x) for x in g("gate_pos").value],
            gate_rpy=[float(x) for x in g("gate_rpy").value],
            drums_enabled=bool(g("drums_enabled").value),
            drum_diameter=float(g("drum_diameter").value),
            drum_depth=float(g("drum_depth").value),
            drum_positions=[float(x) for x in g("drum_positions").value],
            drum_colors=[str(x) for x in g("drum_colors").value],
            flares_enabled=bool(g("flares_enabled").value),
            flare_height=float(g("flare_height").value),
            flare_pole_diameter=float(g("flare_pole_diameter").value),
            flare_base_diameter=float(g("flare_base_diameter").value),
            flare_base_height=float(g("flare_base_height").value),
            flare_ball_diameter=float(g("flare_ball_diameter").value),
            flare_ball_mass=float(g("flare_ball_mass").value),
            flare_positions=[float(x) for x in g("flare_positions").value],
            flare_colors=[str(x) for x in g("flare_colors").value],
            timestep=float(g("timestep").value),
            start_x=float(g("start_x").value),
            start_y=float(g("start_y").value),
            start_z=float(g("start_z").value),
            start_yaw=float(g("start_yaw").value),
        )

    # -- callbacks ----------------------------------------------------------
    def _on_commands(self, msg: Commands):
        # Last-write-wins; mode is ignored in Phase 1 (treated as direct/MANUAL).
        self.cmd = Command(
            arm=bool(msg.arm),
            surge=_clamp_norm(msg.forward),
            sway=_clamp_norm(msg.lateral),
            heave=_clamp_norm(msg.thrust),
            roll=_clamp_norm(msg.roll),
            pitch=_clamp_norm(msg.pitch),
            yaw=_clamp_norm(msg.yaw),
        )

    def _on_physics(self):
        self.sim.step(self.cmd)

    def _on_telemetry(self):
        s = self.sim.get_state()
        roll, pitch, yaw = cv.quaternion_to_euler_zyx(*s["orientation"])  # ENU/REP-103
        wx, wy, wz = s["angular_velocity_body"]

        # Convert to the NED-style ATTITUDE convention reported on /master.
        r_ned, p_ned, y_ned = cv.enu_attitude_to_ned(roll, pitch, yaw)
        rs, ps, ys = cv.enu_rates_to_ned(wx, wy, wz)
        qw, qx, qy, qz = cv.euler_to_quaternion_zyx(r_ned, p_ned, y_ned)

        m = Telemetry()
        m.timestamp = float(s["sim_time"])
        m.battery_voltage = 16.0          # constant (faked)
        m.arm = bool(self.cmd.arm)

        # IMU integer fields are unimplemented placeholders in Phase 1.
        m.imu_gyro_x = m.imu_gyro_y = m.imu_gyro_z = 0
        m.imu_xacc = m.imu_yacc = m.imu_zacc = 0
        m.imu_gyro_compass_x = m.imu_gyro_compass_y = m.imu_gyro_compass_z = 0

        m.q1, m.q2, m.q3, m.q4 = float(qw), float(qx), float(qy), float(qz)
        m.rollspeed, m.pitchspeed, m.yawspeed = float(rs), float(ps), float(ys)
        m.roll, m.pitch, m.yaw = float(r_ned), float(p_ned), float(y_ned)

        m.internal_pressure = 101325.0    # constant (faked)
        # Absolute pressure, exactly invertible to depth by control code.
        m.external_pressure = float(
            101325.0 + self.cfg.water_density * self.cfg.gravity * s["depth"])
        m.heading = cv.heading_from_yaw_ned(y_ned)

        # Cosmetic thruster PWMs (do not affect physics).
        c = self.cmd
        m.thruster_pwms = [
            float(x) for x in cv.allocate_thruster_pwms(
                c.surge, c.sway, c.heave, c.roll, c.pitch, c.yaw)
        ]
        self.telem_pub.publish(m)

    def _on_odometry(self):
        # Clean ENU / REP-103 ground truth for rviz/debug only.
        s = self.sim.get_state()
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"

        pos = s["position"]
        od.pose.pose.position.x = float(pos[0])
        od.pose.pose.position.y = float(pos[1])
        od.pose.pose.position.z = float(pos[2])
        qx, qy, qz, qw = s["orientation"]
        od.pose.pose.orientation.x = float(qx)
        od.pose.pose.orientation.y = float(qy)
        od.pose.pose.orientation.z = float(qz)
        od.pose.pose.orientation.w = float(qw)

        lin = s["linear_velocity_body"]    # twist in child (body) frame per REP-103
        ang = s["angular_velocity_body"]
        od.twist.twist.linear.x = float(lin[0])
        od.twist.twist.linear.y = float(lin[1])
        od.twist.twist.linear.z = float(lin[2])
        od.twist.twist.angular.x = float(ang[0])
        od.twist.twist.angular.y = float(ang[1])
        od.twist.twist.angular.z = float(ang[2])
        self.odom_pub.publish(od)

    def destroy_node(self):
        try:
            self.sim.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
