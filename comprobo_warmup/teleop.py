import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

key_binds = {
    "i": (1.0, 0.0, 0.0, 0.0),
    "k": (0.0, 0.0, 0.0, 0.0),
    "m": (-1.0, 0.0, 0.0, 0.0),
    "j": (0.5, 0.0, 0.0, 1.0),
    "l": (0.5, 0.0, 0.0, -1.0),
    "u": (0.0, 0.0, 0.0, 1.0),
    "o": (0.0, 0.0, 0.0, -1.0)
}

class TeleopPublisher(Node):
    def __init__(self):
        super().__init__("teleop")
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.run_teleop)
    
    def run_teleop(self):
        """
        Watch for key presses and
        Send corresponding velocities
        """
        settings = termios.tcgetattr(sys.stdin)
        key = None
        msg = Twist()
        # TODO: Make control-c work
        while True:
            key = self.getKey(settings)
            if key == '\x03':
                return
            if key in key_binds.keys():
                msg.linear.x = key_binds[key][0]
                msg.linear.y = key_binds[key][1]
                msg.linear.z = key_binds[key][2] 
                msg.angular.z = key_binds[key][3]
                self.pub.publish(msg)

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