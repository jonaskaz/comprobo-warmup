from cmath import inf
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math


class PersonFollowerPublisher(Node):
    def __init__(self):
        super().__init__("teleop")
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.centroid_pub = self.create_publisher(Marker, "visualization_marker", 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.msg = Twist()
        self.follow_distance = 2
        self.p = 1
        self.fov = 20
        self.marker = Marker()
    
    def process_scan(self, scan):
        x, y = self.to_cartesian(scan)
        if len(x) == 0 or len(y) == 0:
            self.set_stop_msg()
        else:
            x_center, y_center = self.get_centroid(x, y)
            self.publish_marker(x_center, y_center)
            theta, mag = self.to_polar(x_center, y_center)
            self.update_msg(theta, mag)
        print(self.msg.linear.x)
        print(self.msg.angular.z)
        self.pub.publish(self.msg)
    
    def set_stop_msg(self):
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0

    def get_valid_fov_indexes(self, scan):
        """
        Neato heading of 0 degrees is aligned with linear x direction
        """
        valid_indexes = []

        # Get the number of scan values that make up the fov
        index_fov = int(len(scan.ranges) / 360 * self.fov)

        # Add the first half of the scan values in the fov
        valid_indexes.append(range(int(index_fov/2)))

        # Add the last half of the scan values in the fov
        valid_indexes.append(range(int(len(scan.ranges) - index_fov/2), len(scan.ranges)))

        return valid_indexes

    def to_cartesian(self, scan):
        """
        sin(theta) = y value / range
        cos(theta) = x value / range
        """
        cartesian_x = []
        cartesian_y = []
        valid_indexes = self.get_valid_fov_indexes(scan)
        for i, v in enumerate(scan.ranges):
            if i in valid_indexes:
                continue
            y = math.sin(scan.angle_increment * i) * v
            x = math.cos(scan.angle_increment * i) * v
            if y != inf and y != -inf and x != inf and x != -inf:
                cartesian_y.append(y)
                cartesian_x.append(x)
        return cartesian_x, cartesian_y
            

    def get_centroid(self, x, y):
        x_mean = sum(x) / len(x)
        y_mean = sum(y) / len(y)
        return x_mean, y_mean
    
    def to_polar(self, x, y):
        theta = math.atan(x/y)
        mag = math.tan(x/y)
        return theta, mag
        
    def update_msg(self, theta, mag):
        self.msg.linear.x = mag
        self.msg.angular.z = theta

    def publish_marker(self, x, y):
        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.id = 0

        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1

        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.centroid_pub.publish(self.marker)
        

def main(args=None):
    rclpy.init(args=args)
    pfp = PersonFollowerPublisher()
    rclpy.spin(pfp)
    rclpy.shutdown()


if __name__ == "__main__":
    main()