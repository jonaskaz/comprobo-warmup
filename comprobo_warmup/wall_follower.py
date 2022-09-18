import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')
        # Create a timer that fires ten times per second
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.cmd_vel = Twist()

        self.max_lin_vel = 0.3   # m/sec
        self.k_lin_vel = 0.4     # deg*m/sec
        self.k_ang_vel = 0.01    # (rad/sec)/deg
        self.desired_angle = 90  # deg

    def process_scan(self, msg):
        """
        Callback function for Laser Scan subscript 0.4
        Takes scan message and sets velocity command where
        forwards velocity is inversely proportional to angular error
        and angular velocity is proportional to angular error

        IN: msg is a LaserScan message
        OUT: Publishes a Twist message
        """
        filtered_scan = self.filter_scan(msg.ranges, msg.range_min, msg.range_max)
        closest_dist = min(filtered_scan)
        closest_ind = filtered_scan.index(closest_dist)  # This index represents degree measurement

        # Compute angular error
        angular_error_deg = self.desired_angle - closest_ind
        if abs(angular_error_deg) > 180:
            if angular_error_deg < 0:
                angular_error_deg = 360 + angular_error_deg
            else:
                angular_error_deg = 360 - angular_error_deg
        print(angular_error_deg)

        # Set linear velocity to inversely proportional to angular error
        if angular_error_deg == 0:
            self.cmd_vel.linear.x = self.max_lin_vel
        else:
            self.cmd_vel.linear.x = self.k_lin_vel / abs(angular_error_deg)

        # Set angular velocity to proportional to angular error (but negative)
        self.cmd_vel.angular.z = - self.k_ang_vel * angular_error_deg

        # Publish velocity command
        self.pub.publish(self.cmd_vel)

    def filter_scan(self, ranges, min_range, max_range):
        """
        Filters the measured ranges of a laser scan, replacing any
        measurement not within the min_ to max_range with infinity

        IN: list of ranges, scalars for min_ & max_range
        OUT: filtered list of ranges, still one range per measurement
        """
        return [dist if (dist>min_range and dist<max_range) else float('inf') for dist in ranges]


def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = WallFollowerNode()  # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()