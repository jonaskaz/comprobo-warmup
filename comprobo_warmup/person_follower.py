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
        self.cartesian_x = []
        self.cartesian_y = []
        self.centroid = [0, 0]
        self.mag = 0.0
        self.theta = 0.0
        self.p = 1
        self.fov = 20
        self.marker = Marker()
    
    def process_scan(self, scan):
        self.to_cartesian(scan)
        if len(self.cartesian_x) <= 0:
            return
        self.get_centroid()
        self.publish_centroid_marker()
        self.centroid_to_polar()
        self.send_follow_vel()
        self.pub.publish(self.msg)

    def to_cartesian(self, scan):
        """
        sin() = y value / range
        cos() = x value / range
        """
        self.cartesian_x = []
        self.cartesian_y = []
        index_fov = int(len(scan.ranges) / 100 * self.fov)
        min_index_max = index_fov/2
        max_index_min = int(len(scan.ranges) - index_fov/2)
        for i, v in enumerate(scan.ranges):
            if i > min_index_max and i < max_index_min:
                continue
            y = math.sin(scan.angle_increment * i) * v
            x = math.cos(scan.angle_increment * i) * v
            if y != inf and y != -inf and x != inf and x != -inf:
                self.cartesian_y.append(y)
                self.cartesian_x.append(x)

    def get_centroid(self):
        self.centroid[0] = sum(self.cartesian_x) / len(self.cartesian_x)
        self.centroid[1] = sum(self.cartesian_y) / len(self.cartesian_y)
    
    def centroid_to_polar(self):
        self.theta = math.atan(self.centroid[0]/self.centroid[1])
        self.mag = math.tan(self.centroid[0]/self.centroid[1])
        
    def send_follow_vel(self):
        print(self.mag)
        print(self.theta)
        self.msg.linear.x = 0.0#self.mag
        self.msg.angular.z = 0.0#self.theta/4

    def publish_centroid_marker(self):
        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.id = 0

        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = self.centroid[0]
        self.marker.pose.position.y = self.centroid[1]
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