"""A Node for controlling the a differential drive robot to map an area."""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import Twist, Pose, PoseStamped, Point, Quaternion

from enum import Enum, auto

import random

import math


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each
    iteration
    """

    STOPPED = auto(),
    MOVING = auto()


class Explore(Node):
    """
    Node for controlling the diff-drive robot to explore an environment.

    Subscribes
    ----------
    map : nav_msgs/msg/OccupancyGrid - the map of the area
    cmd_vel : geometry_msgs/msg/Twist - the speed the robot

    Publishes
    ---------
    goal_pose : geometry_msgs/msg/PoseStamped - where the robot should move

    """

    def __init__(self):
        """Initialize the explore node."""
        super().__init__('explore')
        self.tmr = self.create_timer(1/100, self.timer_callback)

        self.map_sub = self.create_subscription(OccupancyGrid,
                                                'map',
                                                self.map_callback,
                                                10)
        self.cmd_vel_sub = self.create_subscription(Twist,
                                                    'cmd_vel',
                                                    self.vel_callback,
                                                    10)

        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        self.map = None
        self.map_width = 0
        self.map_height = 0
        self.map_res = 0
        self.map_min_x = 0
        self.map_min_y = 0
        self.map_max_x = 0
        self.map_max_y = 0
        self.cmd_vel = None
        self.state = State.STOPPED

    def timer_callback(self):
        """
        Timer callback for controlling the diff-drive robot.

        Publishes commands at a set frequency

        """
        if (self.state == State.STOPPED) and (self.map != None):
            index = random.randint(0, len(self.map.data) - 1)
            if self.map.data[index] == -1:
                goalpose = PoseStamped()
                goalpose.header.frame_id = 'map'
                goalpose.header.stamp = self.get_clock().now().to_msg()
                goalpose.pose = Pose()
                goalpose.pose.position = Point()
                goalpose.pose.orientation = Quaternion()
                # new_x = random.uniform(self.map_min_x, self.map_max_x)
                # new_y = random.uniform(self.map_min_y, self.map_max_y)
                new_x_cell = index % self.map_width
                new_y_cell = math.floor(index / self.map_width)
                new_x = self.map_min_x + new_x_cell * self.map_res
                new_y = self.map_min_y + new_y_cell * self.map_res
                goalpose.pose.position.x = new_x
                goalpose.pose.position.y = new_y
                self.goal_pub.publish(goalpose)

    def map_callback(self, map):
        """
        Update the variables associated with the map.

        Updates variables relating to the current map

        Args:
        ----
        map : (nav_msgs/msg/OccupancyGrid) - the current map

        """
        self.map = map
        self.map_width = map.info.width
        self.map_height = map.info.height
        self.map_res = map.info.resolution
        self.map_min_x = map.info.origin.position.x
        self.map_min_y = map.info.origin.position.y
        self.map_max_x = self.map_min_x + self.map_width * self.map_res
        self.map_max_y = self.map_min_y + self.map_height * self.map_res

    def vel_callback(self, twist):
        """
        Monitor the commanded velocity.

        Updates the current state accordingly

        Args:
        ----
        twist : (geometry_mgs/msg/Twist) - the commanded velocity

        """
        if twist.linear.x == 0.0 and twist.angular.z == 0.0:
            self.state = State.STOPPED
        else:
            self.state = State.MOVING


def main(args=None):
    """Entrypoint for the explore node."""
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
