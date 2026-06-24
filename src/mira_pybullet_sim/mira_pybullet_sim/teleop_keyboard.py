"""Interactive keyboard teleop -> /master/commands (custom_msgs/Commands).

Drive the cuboid from a terminal. PWM convention: 1100-1900, 1500 neutral.
Each keypress nudges one axis; values decay back toward neutral when idle.

Controls:
  W / S : surge forward / back        (forward)
  A / D : sway left / right           (lateral)   A = +Y (left)
  R / F : heave up / down             (thrust)
  Q / E : yaw left / right            (yaw)
  T / G : pitch up / down             (pitch)
  Z / C : roll left / right           (roll)
  SPACE : toggle ARM
  X     : all-stop (neutral, keep arm)
  CTRL-C: quit

Requires a real TTY. Run in its own terminal.
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from custom_msgs.msg import Commands

NEUTRAL = 1500
STEP = 80          # PWM increment per keypress
DECAY = 20         # PWM relax toward neutral per tick
PWM_MIN, PWM_MAX = 1100, 1900
RATE_HZ = 20.0

HELP = __doc__


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.pub = self.create_publisher(Commands, "/master/commands", 10)
        self.axes = {k: NEUTRAL for k in
                     ("forward", "lateral", "thrust", "yaw", "pitch", "roll")}
        self.arm = False
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)
        self._settings = termios.tcgetattr(sys.stdin)
        print(HELP)
        self.get_logger().info("teleop ready (ARM is OFF -- press SPACE to arm)")

    def _bump(self, axis, delta):
        self.axes[axis] = max(PWM_MIN, min(PWM_MAX, self.axes[axis] + delta))

    def _read_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return ""

    def _tick(self):
        key = self._read_key().lower()
        if key:
            if key == " ":
                self.arm = not self.arm
                self.get_logger().info(f"ARM -> {self.arm}")
            elif key == "x":
                for a in self.axes:
                    self.axes[a] = NEUTRAL
            elif key == "w": self._bump("forward", STEP)
            elif key == "s": self._bump("forward", -STEP)
            elif key == "a": self._bump("lateral", STEP)
            elif key == "d": self._bump("lateral", -STEP)
            elif key == "r": self._bump("thrust", STEP)
            elif key == "f": self._bump("thrust", -STEP)
            elif key == "q": self._bump("yaw", STEP)
            elif key == "e": self._bump("yaw", -STEP)
            elif key == "t": self._bump("pitch", STEP)
            elif key == "g": self._bump("pitch", -STEP)
            elif key == "z": self._bump("roll", STEP)
            elif key == "c": self._bump("roll", -STEP)
        else:
            # Decay toward neutral.
            for a, v in self.axes.items():
                if v > NEUTRAL:
                    self.axes[a] = max(NEUTRAL, v - DECAY)
                elif v < NEUTRAL:
                    self.axes[a] = min(NEUTRAL, v + DECAY)

        self._publish()

    def _publish(self):
        m = Commands()
        m.arm = self.arm
        m.mode = "MANUAL"
        m.forward = int(self.axes["forward"])
        m.lateral = int(self.axes["lateral"])
        m.thrust = int(self.axes["thrust"])
        m.yaw = int(self.axes["yaw"])
        m.pitch = int(self.axes["pitch"])
        m.roll = int(self.axes["roll"])
        m.servo1 = NEUTRAL
        m.servo2 = NEUTRAL
        self.pub.publish(m)

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        tty.setcbreak(sys.stdin.fileno())
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
