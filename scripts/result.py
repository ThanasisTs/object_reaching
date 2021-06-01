#!/usr/bin/env python
import rospy
from std_msgs.msg import Time

robot_time, human_time = None, None
robot_time_flag, human_time_flag = False, False

def robot_time_callback(msg):
	global robot_time, robot_time_flag
	robot_time = msg.data.to_sec()
	robot_time_flag = True

def human_time_callback(msg):
	global human_time, human_time_flag
	human_time = msg.data.to_sec()
	human_time_flag = True

if __name__ == "__main__":
	rospy.init_node("result")

	# Subscribes to the time that the robot hits the object
	robot_sub = rospy.Subscriber('robot_time_topic', Time, robot_time_callback)

	# Subscribes to the time that the human reaches the object
	human_sub = rospy.Subscriber('human_time_topic', Time, human_time_callback)

	# Check who won (human or robot)
	while not rospy.is_shutdown():
		if human_time_flag and robot_time_flag:
			rospy.logerr("Human won by {} secs".format((robot_time-human_time)*1000)) if robot_time-human_time > 0 else rospy.logwarn("Robot won by {} secs".format((human_time-robot_time)*1000))
			human_time_flag, robot_time_flag = False, False
			