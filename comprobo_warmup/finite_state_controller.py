import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from enum import Enum


class State(Enum):
    """
    Enum to track the state of the robot
    """
    CIRCLE = 0
    WALL_FOLLOW = 1


class FiniteStateControllerNode(Node):
    """
    Controller for driving the robot in a circle and
    following the nearest wall within a range
    """
    def __init__(self):
        super().__init__('finite_state_controller')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_timer(0.1, self.run_loop)
        self.state = State.CIRCLE
        self.msg = Twist()
        self.max_lin_vel = 0.3   # m/sec
        self.k_lin_vel = 0.7    # deg*m/sec
        self.k_ang_vel = 0.01    # (rad/sec)/deg
        self.desired_angle = 90  # deg
        self.scan = LaserScan()
        self.wall_range = 0.05
        self.max_wall_distance = 0.9

    def process_scan(self, scan):
        """
        Check if a wall is found
        Transition self.state if a wall is found
        """
        self.scan = scan
        if self.wall_found():
            self.state = State.WALL_FOLLOW
        else:
            self.state = State.CIRCLE

    def run_loop(self):
        """
        Check self.state
        Run the corresponding drive loop
        """
        if self.state == State.CIRCLE:
            self.drive_circle()
        elif self.state == State.WALL_FOLLOW:
            self.follow_wall()

    def wall_found(self) -> bool:
        """
        Check if there are 5 points next to each other within wall_range of each other
        """
        found = []
        filtered_scan = self.filter_scan(self.scan.ranges, self.scan.range_min, self.max_wall_distance)
        for i, s in enumerate(filtered_scan):
            if found == []:
                found.append(s)
            else:
                middle = sum(found) / len(found)
                if s < middle +self.wall_range and s > middle - self.wall_range:
                    found.append(s)
                else:
                    found = []
            if len(found) == 5:
                return True
        return False

    def drive_circle(self):
        """
        Drive the robot in a circle
        """
        self.msg.linear.x = 0.5
        self.msg.angular.z = 0.7
        self.pub.publish(self.msg)

    def follow_wall(self):
        """
        Takes scan message and sets velocity command where
        forwards velocity is inversely proportional to angular error
        and angular velocity is proportional to angular error

        OUT: Publishes a Twist message
        """
        filtered_scan = self.filter_scan(self.scan.ranges, self.scan.range_min, self.max_wall_distance)
        closest_dist = min(filtered_scan)
        closest_ind = filtered_scan.index(closest_dist)  # This index represents degree measurement

        # Compute angular error
        angular_error_deg = self.desired_angle - closest_ind
        if abs(angular_error_deg) > 180:
            if angular_error_deg < 0:
                angular_error_deg = 360 + angular_error_deg
            else:
                angular_error_deg = 360 - angular_error_deg

        # Set linear velocity to inversely proportional to angular error
        if angular_error_deg == 0:
            self.msg.linear.x = self.max_lin_vel
        else:
            self.msg.linear.x = self.k_lin_vel / abs(angular_error_deg)

        # Set angular velocity to proportional to angular error (but negative)
        self.msg.angular.z = - self.k_ang_vel * angular_error_deg

        # Publish velocity command
        self.pub.publish(self.msg)

    def filter_scan(self, ranges, min_range, max_range):
        """
        Filters the measured ranges of a laser scan, replacing any
        measurement not within the min_ to max_range with infinity

        IN: list of ranges, scalars for min_ & max_range
        OUT: filtered list of ranges, still one range per measurement
        """
        return [dist if (dist>min_range and dist<max_range) else float('inf') for dist in ranges]


def main(args=None):
    rclpy.init(args=args)      
    fscn = FiniteStateControllerNode() 
    rclpy.spin(fscn)         
    rclpy.shutdown()

if __name__ == '__main__':
    main()