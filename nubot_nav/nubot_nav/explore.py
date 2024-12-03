import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import Twist, Pose, PoseStamped, Point, Quaternion

from enum import Enum, auto

import random

import math

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    STOPPED = auto(),
    MOVING = auto()

class Explore(Node):

    def __init__(self):
        super().__init__('explore')
        self.tmr = self.create_timer(1/100, self.timer_callback)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.vel_callback, 10)
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
        if (self.state == State.STOPPED) and (self.map != None):
            goalpose = PoseStamped()
            goalpose.header.frame_id = 'map'
            goalpose.header.stamp = self.get_clock().now().to_msg()
            goalpose.pose = Pose()
            goalpose.pose.position = Point()
            goalpose.pose.orientation = Quaternion()
            new_x = random.uniform(self.map_min_x, self.map_max_x)
            new_y = random.uniform(self.map_min_y, self.map_max_y)
            goalpose.pose.position.x = new_x
            goalpose.pose.position.y = new_y
            # angle = math.atan2(self.map_min_y - new_y, self.map_min_x - new_x)
            # goalpose.pose.orientation.w = math.cos(angle * .5)
            # goalpose.pose.orientation.z = math.sin(angle * .5)
            self.goal_pub.publish(goalpose)
        # elif self.state == State.MOVING:
        #     pass
        # pass

    def map_callback(self, map):
        self.map = map
        self.map_width = map.info.width
        self.map_height = map.info.height
        self.map_res = map.info.resolution
        self.map_min_x = map.info.origin.position.x
        self.map_min_y = map.info.origin.position.y
        self.map_max_x = self.map_min_x + self.map_width * self.map_res
        self.map_max_y = self.map_min_y + self.map_height * self.map_res

    def vel_callback(self, twist):
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
