import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DriveSquarePublisher(Node):
    def __init__(self):
        super().__init__("drive_square")
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # TODO: Find a better way to run this than a timer
        self.create_timer(30, self.drive_square)

    def drive_square(self):
        msg = Twist()
        self.stop(msg)
        self.straight(msg)
        self.turn_90(msg)
        self.straight(msg)
        self.turn_90(msg)
        self.straight(msg)
        self.turn_90(msg)
        self.straight(msg)
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


def main(args=None):
    rclpy.init(args=args)
    drive_square_publisher = DriveSquarePublisher()
    rclpy.spin(drive_square_publisher)
    rclpy.shutdown()


if __name__ == "__main__":
    main()