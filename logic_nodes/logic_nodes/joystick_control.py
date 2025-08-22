import rclpy
from rclpy.node import Node
import sys, termios, tty, select

from custom_interfaces.msg import Pose


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Pose, 'orient_ctrl', 10)

        self.yaw = 0.0
        self.pitch = 0.0
        self.step = 5.0  # degrees to increment per keypress

        self.get_logger().info("Keyboard Teleop Started")
        self.get_logger().info("Use keys: w/s = pitch up/down, a/d = yaw left/right, q = quit")

    def get_key(self):
        """Non-blocking key press reader"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            if key == 'w':
                self.pitch += self.step
            elif key == 's':
                self.pitch -= self.step
            elif key == 'a':
                self.yaw += self.step
            elif key == 'd':
                self.yaw -= self.step
            elif key == 'q':
                self.get_logger().info("Exiting teleop...")
                break

            # Publish the Pose
            msg = Pose()
            msg.yaw_theta = self.yaw
            msg.pitch_theta = self.pitch
            self.publisher_.publish(msg)

            self.get_logger().info(f"Published -> yaw: {self.yaw}, pitch: {self.pitch}")


def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
