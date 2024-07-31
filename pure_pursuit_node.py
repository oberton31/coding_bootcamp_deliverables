#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv

import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry


# To do, takes first two turns correctly, but not third one, debug that

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('file_path', "/home/oberton/sim_ws/sim_waypoints/07-28-21-05-56.csv")
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('lookahead', 0.5)
        self.drive_pub_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.odom_sub_ = self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)

        file_path_ = str(self.get_parameter('file_path').value)
        self.waypoints_ = self.read_csv(file_path_)

        self.current_position_ = None
        self.waypoint_index_ = 0

    def pose_callback(self, pose_msg):
        lookahead_distance = self.get_parameter('lookahead').value
        self.current_position_ = (pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y)

        waypoint = self.find_best_waypoint(lookahead_distance)

        if waypoint is None: # reached end
            final_msg = AckermannDriveStamped()
            final_msg.drive.steering_angle = 0.0
            final_msg.drive.speed = 0.0
            self.drive_pub_.publish(final_msg)
        else:
            alpha, distance = self.transform_coords(waypoint, pose_msg)

            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.get_parameter('speed').value

            if distance == 0:
                steering_angle = 0.0
            else:
                steering_angle = math.atan2(2 * lookahead_distance * math.sin(alpha) , distance) # found this online
            
            max_steering_angle = np.pi/4

            steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle)) # limit steering angle to -pi/4 to pi/4

            drive_msg.drive.steering_angle = steering_angle

            self.drive_pub_.publish(drive_msg)

    def transform_coords(self, waypoint, pose_msg): # put coords in world frame in robots reference frame
        orientation = pose_msg.pose.pose.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        theta = math.atan2(waypoint[1] - self.current_position_[1], waypoint[0] - self.current_position_[0])# angle between current point and waypoint (draw triange with y and x and find angle)
        angle_difference = theta - yaw # difference between current angle of robot and angle it needs to traverse to reach next point

        distance = self.calc_distance(self.current_position_[0], self.current_position_[1], waypoint[0], waypoint[1])

        # this gives the waypoint in the pov of the rover, taking into account its current trajectory
        return angle_difference, distance #distance * math.cos(angle_difference), distance * math.sin(angle_difference), distance

    def find_best_waypoint(self, lookahead_distance):
        if self.waypoint_index_ >= len(self.waypoints_) - 1: # if no more waypoints, return None and later keep the robot stationary
            return None
        
        min_difference = float('inf')
        waypoint_x = 0.0
        waypoint_y = 0.0
        end_position = len(self.waypoints_)
        if  len(self.waypoints_) > self.waypoint_index_ + int(len(self.waypoints_) / 8):
            end_position = self.waypoint_index_ + int(len(self.waypoints_) / 8) # only search through immediate waypoints in front of robot
        
        # search through waypoints, find waypoint closest to being 1 unit away from current position
        for i in range(self.waypoint_index_, end_position):
            x = self.waypoints_[i][0]
            y = self.waypoints_[i][1]
            distance = self.calc_distance(x, y, self.current_position_[0], self.current_position_[1])
            difference = abs(distance - lookahead_distance)
            if difference < min_difference:
                min_difference = difference
                waypoint_x = x
                waypoint_y = y
                self.waypoint_index_ = i

        return (waypoint_x, waypoint_y)
        
    def calc_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def read_csv(self, file_path):
        data = []
        with open(file_path, 'r') as file:
            csv_reader = csv.reader(file)
            for line in csv_reader:
                data.append([float(val) for val in line])
        return data
    
    

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    try:
        rclpy.spin(pure_pursuit_node)
    except:
        print("Shutting Down")
    finally:
        pure_pursuit_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
