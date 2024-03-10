#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion
import numpy as np  
from visualization_msgs.msg import Marker

class PathFollower:
    def __init__(self):
        rospy.init_node('path_follower', anonymous=True)
        
        self.desired_path = None
        self.current_pose = None
        
        self.cmd_vel_pub = rospy.Publisher('/bluerov2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/path', Path, self.desired_path_callback)
        rospy.Subscriber('/bluerov2/pose_gt', Odometry, self.current_pose_callback)
        self.next_marker_pub = rospy.Publisher('next_point_marker', Marker, queue_size=10)
        self.robot_marker_pub = rospy.Publisher('robot_point_marker', Marker, queue_size=10)
        
        self.GOAL_TOLERANCE = 5
        self.LINEAR_GAIN = 3.0
        self.TURNING_GAIN = 0.03

        self.rate = rospy.Rate(10) # 10 Hz

    def desired_path_callback(self, data):
        self.desired_path = data

    def current_pose_callback(self, data):
        self.current_pose = data.pose.pose

    def follow_path(self):
        while not rospy.is_shutdown():
            if self.desired_path is not None and self.current_pose is not None:
                current_position = np.array([
                    self.current_pose.position.x,
                    self.current_pose.position.y
                ])
                current_orientation = self.current_pose.orientation
                _, _, current_yaw = euler_from_quaternion([
                    current_orientation.x,
                    current_orientation.y,
                    current_orientation.z,
                    current_orientation.w
                ])

                # Find the closest point on the path
                closest_point_idx = None
                closest_point_distance = float('inf')
                for i, pose in enumerate(self.desired_path.poses):
                    path_point = np.array([
                        pose.pose.position.x,
                        pose.pose.position.y
                    ])
                    distance = np.linalg.norm(current_position - path_point)
                    if distance < closest_point_distance:
                        closest_point_distance = distance
                        closest_point_idx = i
                
                # Remove points within distance L from the current robot position
                filtered_path = self.desired_path.poses[closest_point_idx:]
                filtered_path = [pose.pose.position for pose in filtered_path if np.linalg.norm(current_position - np.array([
                    pose.pose.position.x,
                    pose.pose.position.y
                ])) > self.GOAL_TOLERANCE]

                if len(filtered_path) > 1:
                    next_point = np.array([
                        filtered_path[1].x,
                        filtered_path[1].y
                    ])
                    
                    self.publish_next_goal_marker(next_point)
                    self.publish_current_robot_marker(current_position)
                    
                    # Calculate desired angle
                    desired_angle = np.arctan2(next_point[1] - current_position[1], next_point[0] - current_position[0])
                    # Calculate error between current and desired angle
                    angle_error = desired_angle - current_yaw

                    # Calculate desired linear and angular velocities
                    dx = next_point[0] - current_position[0]
                    dy = next_point[1] - current_position[1]
                    dx_body = dx*np.cos(current_yaw)+dy*np.sin(current_yaw)
                    dy_body = -dx*np.sin(current_yaw)+dy*np.cos(current_yaw)
                    distance_to_goal = np.linalg.norm(next_point - current_position)

                    desired_linear_velocity_x = self.LINEAR_GAIN * (dx_body)
                    desired_linear_velocity_y = self.LINEAR_GAIN * (dy_body)
                    desired_angular_velocity = self.TURNING_GAIN * angle_error

                    cmd_vel_msg = Twist()
                    cmd_vel_msg.linear.x = desired_linear_velocity_x
                    cmd_vel_msg.linear.y = desired_linear_velocity_y
                    cmd_vel_msg.angular.z = desired_angular_velocity

                    self.cmd_vel_pub.publish(cmd_vel_msg)
            self.rate.sleep()
            
    def publish_next_goal_marker(self, next_point):        
        next_marker = Marker()
        next_marker.header.frame_id = "map"
        next_marker.type = Marker.SPHERE
        next_marker.action = Marker.ADD
        next_marker.scale.x = 2
        next_marker.scale.y = 2
        next_marker.scale.z = 2
        next_marker.color.a = 1.0  # Alpha (transparency)
        next_marker.color.r = 0.0  # Red
        next_marker.color.g = 1.0  # Green
        next_marker.color.b = 0.0  # Blue

        next_marker.pose.orientation.w = 1.0
        next_marker.pose.position.x = next_point[0]
        next_marker.pose.position.y = next_point[1]
        next_marker.pose.position.z = 0.0

        self.next_marker_pub.publish(next_marker)

    def publish_current_robot_marker(self, current_position):
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.scale.x = 2
        robot_marker.scale.y = 2
        robot_marker.scale.z = 2
        robot_marker.color.a = 1.0  # Alpha (transparency)
        robot_marker.color.r = 1.0  # Red
        robot_marker.color.g = 0.0  # Green
        robot_marker.color.b = 0.0  # Blue
        robot_marker.pose.orientation.w = 1.0
        robot_marker.pose.position.x = current_position[0]
        robot_marker.pose.position.y = current_position[1]
        robot_marker.pose.position.z = 0.0

        self.robot_marker_pub.publish(robot_marker)

if __name__ == '__main__':
    try:
        path_follower = PathFollower()
        path_follower.follow_path()
    except rospy.ROSInterruptException:
        pass
