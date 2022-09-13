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
        self.msg = Twist()
        

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    tp = TeleopPublisher()
    settings = termios.tcgetattr(sys.stdin)
    key = None
    while key != '\x03':
        key = getKey(settings)
        if key in key_binds.keys():
            tp.msg.linear.x = key_binds[key][0]
            tp.msg.linear.y = key_binds[key][1]
            tp.msg.linear.z = key_binds[key][2] 
            tp.msg.angular.z = key_binds[key][3]
        tp.pub.publish(tp.msg)
    rclpy.shutdown()


if __name__ == "__main__":
    main()