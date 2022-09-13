import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DriveSquarePublisher(Node):
    """
    Node for driving neato in a 1m by 1m square
    """
    def __init__(self):
        super().__init__("drive_square")
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def drive_square(self):
        """
        Drive in a 1m by 1m square
        """
        msg = Twist()
        self.stop(msg)
        for i in range(4):
            self.stop(msg)
            self.straight(msg)
            self.stop(msg)
            self.turn_90(msg)
        self.stop(msg)
    
    def straight(self, msg):
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        time.sleep(1)
    
    def turn_90(self, msg):
        msg.linear.x = 0.0
        msg.angular.z = -0.5
        self.pub.publish(msg)
        time.sleep(3)
    
    def stop(self, msg):
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    dsp = DriveSquarePublisher()
    dsp.drive_square()
    rclpy.shutdown()


if __name__ == "__main__":
    main()