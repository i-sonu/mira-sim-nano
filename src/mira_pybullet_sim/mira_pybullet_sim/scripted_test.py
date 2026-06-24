"""Non-interactive scripted command sequence -> /master/commands.

Publishes a timed sequence so you can watch the cuboid move without a keyboard.
Each phase holds one channel; channels return to neutral between phases so you can
confirm each DOF moves independently. PWM convention: 1100-1900, 1500 neutral.

Run after launching the sim; watch /bluerov2/odometry or /master/telemetry.
"""

import rclpy
from rclpy.node import Node
from custom_msgs.msg import Commands

NEUTRAL = 1500
HI, LO = 1900, 1100

# (label, duration_s, arm, {channel: pwm})
SEQUENCE = [
    ("settle (disarmed)",        3.0, False, {}),
    ("arm + neutral",            2.0, True,  {}),
    ("surge forward",            3.0, True,  {"forward": HI}),
    ("neutral",                  2.0, True,  {}),
    ("sway left",                3.0, True,  {"lateral": HI}),
    ("neutral",                  2.0, True,  {}),
    ("heave down",               3.0, True,  {"thrust": LO}),
    ("neutral",                  2.0, True,  {}),
    ("yaw",                      3.0, True,  {"yaw": HI}),
    ("neutral",                  2.0, True,  {}),
    ("pitch",                    3.0, True,  {"pitch": HI}),
    ("neutral",                  2.0, True,  {}),
    ("roll",                     3.0, True,  {"roll": HI}),
    ("neutral",                  2.0, True,  {}),
    ("disarm",                   3.0, False, {}),
]

RATE_HZ = 20.0


class ScriptedTest(Node):
    def __init__(self):
        super().__init__("scripted_test")
        self.pub = self.create_publisher(Commands, "/master/commands", 10)
        self.idx = 0
        self.elapsed = 0.0
        self.dt = 1.0 / RATE_HZ
        self.timer = self.create_timer(self.dt, self._tick)
        self._announce()

    def _announce(self):
        label = SEQUENCE[self.idx][0]
        self.get_logger().info(f"[{self.idx + 1}/{len(SEQUENCE)}] {label}")

    def _tick(self):
        if self.idx >= len(SEQUENCE):
            self.get_logger().info("sequence complete; disarming and exiting")
            self._publish(False, {})
            rclpy.shutdown()
            return

        _, dur, arm, chans = SEQUENCE[self.idx]
        self._publish(arm, chans)
        self.elapsed += self.dt
        if self.elapsed >= dur:
            self.elapsed = 0.0
            self.idx += 1
            if self.idx < len(SEQUENCE):
                self._announce()

    def _publish(self, arm, chans):
        m = Commands()
        m.arm = arm
        m.mode = "MANUAL"
        m.forward = chans.get("forward", NEUTRAL)
        m.lateral = chans.get("lateral", NEUTRAL)
        m.thrust = chans.get("thrust", NEUTRAL)
        m.yaw = chans.get("yaw", NEUTRAL)
        m.pitch = chans.get("pitch", NEUTRAL)
        m.roll = chans.get("roll", NEUTRAL)
        m.servo1 = NEUTRAL
        m.servo2 = NEUTRAL
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ScriptedTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
