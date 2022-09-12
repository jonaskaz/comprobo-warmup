import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__("teleop")
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.run_teleop)
    
    def run_teleop(self):
        """
        Wait for key presses
        Get the key
        Send the relevant Twist command
        """
        settings = termios.tcgetattr(sys.stdin)
        key = None
        msg = Twist()
        while key := '\x03':
            key = self.getKey(settings)
            if key == "i":
                msg = Twist()
                msg.linear.x = 1.0
                self.pub.publish(msg)
            if key == "m":
                msg = Twist()
                msg.linear.x = -1.0
                self.pub.publish(msg)
        return

    @staticmethod
    def getKey(settings):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


def main(args=None):
    rclpy.init(args=args)
    teleop_publisher = TeleopPublisher()
    rclpy.spin(teleop_publisher)
    rclpy.shutdown()


if __name__ == "__main__":
    main()