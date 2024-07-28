#!/usr/bin/env python

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from time import strftime, gmtime
import os

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')

        home_dir = os.path.expanduser("~")
        log_dir = os.path.join(home_dir, 'sim_ws', 'sim_waypoints')
        file_path = os.path.join(log_dir, strftime('%m-%d-%H-%M-%S', gmtime()) + '.csv')
        self.file = open(file_path, 'w')

        self.odometry_sub_ = self.create_subscription(
            Odometry, 
            'ego_racecar/odom', 
            self.waypoint_logger_callback, 
            10
        )

    def waypoint_logger_callback(self, data):
        # Log the data to the file
        self.file.write('%f, %f, %f, %f, %f, %f, %f\n' % (
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z,
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ))

    def destroy_node(self):
        if not self.file.closed:
            self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()

    try:
        rclpy.spin(waypoint_logger)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_logger.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()