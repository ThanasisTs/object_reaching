#!/usr/bin/env python
import rospy
from std_msgs.msg import Time

third_pixel_time, prediction_time, robot_motion_start_time = None, None, None
third_pixel_flag, prediction_flag, robot_motion_start_flag = False, False, False

def third_pixel_callback(msg):
	global third_pixel_time, third_pixel_flag
	third_pixel_time = msg.data.to_sec()
	third_pixel_flag = True

def prediction_time_callback(msg):
	global prediction_time, prediction_flag
	prediction_time = msg.data.to_sec()
	prediction_flag = True

def robot_motion_start_callback(msg):
	global robot_motion_start_time, robot_motion_start_flag
	robot_motion_start_time = msg.data.to_sec()
	robot_motion_start_flag = True

if __name__ == "__main__":
	rospy.init_node("check_timestamps")
	third_pixel_sub = rospy.Subscriber('third_pixel_topic', Time, third_pixel_callback)
	prediction_time_sub = rospy.Subscriber('prediction_time_topic', Time, prediction_time_callback)
	robot_motion_start_sub = rospy.Subscriber('robot_motion_start_topic', Time, robot_motion_start_callback)

	while not (robot_motion_start_flag and third_pixel_flag and prediction_flag):
		pass

	prediction_time -= third_pixel_time
	robot_motion_start_time -= third_pixel_time
	third_pixel_time = 0
	# rospy.loginfo("Human won by {} secs".format(robot_time-human_time)) if robot_time-human_time > 0 else rospy.loginfo("Robot won by {} secs".format(human_time-robot_time))
	rospy.loginfo("Third pixel time: {}".format(third_pixel_time))
	rospy.loginfo("Prediction time: {}".format(prediction_time))
	rospy.loginfo("Robot motion start time: {}".format(robot_motion_start_time))

	rospy.spin()