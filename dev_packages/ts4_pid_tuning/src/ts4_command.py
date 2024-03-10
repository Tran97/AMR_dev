#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler
import math

def ts4_command():
    rospy.init_node('ts4_command', anonymous=True)
    pose_pub = rospy.Publisher('/bluerov2/cmd_pose', PoseStamped, queue_size=10)
    twist_pub = rospy.Publisher('/bluerov2/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Initial linear x movement at 1.5 m/s for 10 seconds
    twist_msg = Twist()
    twist_msg.linear.x = 1.5

    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < 10.0:
        twist_pub.publish(twist_msg)
        rate.sleep()
    
    twist_msg = Twist()
    twist_pub.publish(twist_msg)

    while not rospy.is_shutdown():

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 10.0:
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            #pose_msg.header.frame_id = "base_link"

            yaw_degrees = 20
            yaw_radians = math.radians(yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw_radians)

            pose_msg.pose.position.x = -1
            pose_msg.pose.position.y = 0
            pose_msg.pose.position.z = -34.5

            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            pose_pub.publish(pose_msg)

            rate.sleep()

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 10.0:
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            #pose_msg.header.frame_id = "base_link"

            yaw_degrees = -20
            yaw_radians = math.radians(yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw_radians)

            pose_msg.pose.position.x = -1
            pose_msg.pose.position.y = 0
            pose_msg.pose.position.z = -34.5

            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            pose_pub.publish(pose_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        ts4_command()
    except rospy.ROSInterruptException:
        pass
