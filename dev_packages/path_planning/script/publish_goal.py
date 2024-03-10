#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def publish_goal():
    rospy.init_node('publish_goal_node', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    goal_msg = PoseStamped()
    goal_msg.header.frame_id = "map"  # Assuming the frame_id of the goal is "map"
    
    # Set the goal position from parameters
    goal_msg.pose.position.x = rospy.get_param('~goal_x', 100.0)
    goal_msg.pose.position.y = rospy.get_param('~goal_y', 200.0)
    goal_msg.pose.position.z = rospy.get_param('~goal_z', 0.0)

    # Set the goal orientation (adjust as needed)
    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = 0.0
    goal_msg.pose.orientation.w = 1.0

    rospy.loginfo("Publishing goal...")
    while not rospy.is_shutdown():
        pub.publish(goal_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass
