import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoid')
        # Create a timer that fires ten times per second
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.cmd_vel = Twist()

    def process_scan(self, msg):
        """
        TODO: Implement obstacle avoidance from laser scan
        """
        filtered_scan = self.filter_scan(msg.ranges, msg.range_min, msg.range_max)
        pass

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
    node = ObstacleAvoidNode()  # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()