import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


settings = termios.tcgetattr(sys.stdin)
key = None

while key != '\x03':
    key = getKey()
    print(key)


class TeleopPublisher(Node):
    def __init__(self):
        super().__init__("teleop")
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def run_teleop(self):
        """
        Wait for key presses
        Get the key
        Send the relevant Twist command
        """

    @staticmethod
    def getKey():
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