from cmath import inf
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math


class PersonFollowerPublisher(Node):
    """
    Find the nearest "person-like object" 
    Direct the neato to follow it at a given distance
    """
    def __init__(self):
        super().__init__("person_follower")
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.centroid_pub = self.create_publisher(Marker, "visualization_marker", 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_timer(0.1, self.run_loop)
        self.msg = Twist()
        self.follow_distance = 0.2
        self.max_person_distance = 1
        self.p_linear = 0.7
        self.p_angular = 0.7
        self.fov = 50
        self.mag = 0.0
        self.theta = 0.0
    
    def run_loop(self):
        self.update_vel_msg()
        self.pub.publish(self.msg)

    def update_vel_msg(self):
        """
        Update the cmd_vel message using 
        mag, theta and their corresponding multipliers
        """
        self.msg.linear.x = self.mag * self.p_linear
        self.msg.angular.z = self.theta * self.p_angular
    
    def process_scan(self, scan):
        """
        Process the laser scan
        Update self.theta and self.mag
        """
        x, y = self.to_cartesian(scan)
        if len(x) == 0 or len(y) == 0:
            self.mag = 0.0
            self.theta = 0.0
        else:
            x_center, y_center = self.get_centroid(x, y)
            self.publish_marker(x_center, y_center)
            theta, mag = self.to_polar(x_center, y_center)
            theta, mag = self.set_follow_distance(theta, mag)
            self.theta = theta
            self.mag = mag

    def set_follow_distance(self, theta, mag):
        """
        Reduce the magnitude to account for follow distance
        """
        mag -= self.follow_distance
        if mag < 0:
            mag = 0.0
            theta = 0.0
        return theta, mag
    
    def set_stop_msg(self):
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0

    def get_valid_fov_indexes(self, scan):
        """
        Neato heading of 0 degrees is aligned with linear x direction
        Returns scan indexes within the fov
        """
        valid_indexes = []

        # Get the number of scan values that make up the fov
        index_fov = int(len(scan.ranges) / 360 * self.fov)

        # Add the first half of the scan values in the fov
        valid_indexes.extend(range(int(index_fov/2)))

        # Add the last half of the scan values in the fov
        valid_indexes.extend(range(int(len(scan.ranges) - index_fov/2), len(scan.ranges)))

        return valid_indexes

    def to_cartesian(self, scan):
        """
        Convert scan values from polar to cartesian
        sin(theta) = y value / scan
        cos(theta) = x value / scan
        """
        cartesian_x = []
        cartesian_y = []
        valid_indexes = self.get_valid_fov_indexes(scan)
        for i, v in enumerate(scan.ranges):
            if i not in valid_indexes:
                continue
            if v > self.max_person_distance or v == 0:
                continue
            y = math.sin(scan.angle_increment * i) * v
            x = math.cos(scan.angle_increment * i) * v
            if y != inf and y != -inf and x != inf and x != -inf:
                cartesian_y.append(y)
                cartesian_x.append(x)
        return cartesian_x, cartesian_y
            

    def get_centroid(self, x, y):
        """
        Calculate the mean of x and y arrays
        """
        x_mean = sum(x) / len(x)
        y_mean = sum(y) / len(y)
        return x_mean, y_mean
    
    def to_polar(self, x, y):
        """
        Convert x and y from cartesian to polar coordinates
        """
        if y == 0:
            if x > 0:
                theta = math.pi / 2
            elif x < 0:
                theta = 3 * math.pi / 2
            elif x == 0:
                theta = 0
            mag = x
        else:
            theta = math.atan(x/y)
            mag = math.sqrt(x**2 + y**2)
        return theta, mag

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.centroid_pub.publish(marker)
        

def main(args=None):
    rclpy.init(args=args)
    pfp = PersonFollowerPublisher()
    rclpy.spin(pfp)
    rclpy.shutdown()


if __name__ == "__main__":
    main()