#!/usr/bin/env python
import rospy
from std_msgs.msg import Time, Bool
from openpose_ros_msgs.msg import OpenPoseHumanList
from geometry_msgs.msg import Vector3Stamped
from std_srvs.srv import Empty, Trigger, EmptyResponse

import numpy as np


start, end, start_std, print_time = False, False, False, True
pub, time_pub = None, None
x_prev, y_prev, time_prev, x_motion, y_motion, time_motion = [], [], [], [], [], []
count = 0
start_time = None
prediction_pixel_time = None
start_game, reset_game = False, True


# Service callback to initiate human motion detection module
def service(req):
	global start_game
	start_game = True
	rospy.logwarn("Ready to record human movement")
	return EmptyResponse()

# Reset the game. Initialize corresponding variables
def reset_game_callback(msg):
	global reset_game
	rospy.logwarn('Reset the game. Waiting for signal to record human motion...')
	reset_game = msg.data


# Removal of NaNs, outliers and redundant points at the start of the motion
def openpose_callback(msg):
	global pub, prediction_pixel_time, prediction_pixel_pub, time_pub, start_game, start, end, start_std, count, x_prev, y_prev, time_prev, start_time, print_time

	if start_game:
		try:
			x = msg.human_list[0].body_key_points_with_prob[4].x
			y = msg.human_list[0].body_key_points_with_prob[4].y
		except:
			return
		time = msg.header.stamp
		count += 1
		if x != 0 and y != 0:
			if len(x_prev) == 0:
				x_prev.append(x)
				y_prev.append(y)
				time_prev.append(time)
				if start:
					x_motion.append(x)
					y_motion.append(y)
					time_motion.append(time)
					pixel = Vector3Stamped()
					pixel.vector.x = x
					pixel.vector.y = y
					pixel.header.stamp = msg.header.stamp
					pub.publish(pixel)
			else:
				if len(x_prev) == 10:
					start_std = True
				if abs(x - x_prev[-1]) < 30 and abs(y - y_prev[-1]) < 30:
					if start:
						if len(x_prev) == 5:
							del x_prev[0], y_prev[0], time_prev[0]
					elif len(x_prev) == 10:
						del x_prev[0], y_prev[0], time_prev[0]
					x_prev.append(x)
					y_prev.append(y)
					time_prev.append(time)
					if start_std:
						std_x = np.std(x_prev)
						std_y = np.std(y_prev)
						std_total = np.sqrt(std_x**2 + std_y**2)
						if (not start) and (std_total > 1.5):
							print("Motion started at sample {}".format(count))
							start_time = rospy.Time.now().to_sec()
							x_prev, y_prev, time_prev = [], [], []
							start = True
						if start:
							if std_total > 10:
								end = True
							if std_total <= 10 and end:
								if print_time:
									end_time = rospy.Time.now().to_sec()
									# rospy.loginfo("Human motion duration: {}".format((end_time - start_time)*1000))
									# rospy.loginfo("Human motion duration from the prediction onwards: {}".format((end_time - prediction_pixel_time.data.to_sec())*1000))
									print("Motion ended at sample {}".format(count))
									human_time = Time()
									human_time.data = rospy.Time.now()
									time_pub.publish(human_time)
									print_time = False
								return
							x_motion.append(x)
							y_motion.append(y)
							if len(x_motion) == 4:
								prediction_pixel_time = Time()
								prediction_pixel_time.data = rospy.Time.now()
								prediction_pixel_pub.publish(prediction_pixel_time)
							time_motion.append(time)
							pixel = Vector3Stamped()
							pixel.vector.x = x
							pixel.vector.y = y
							pixel.header.stamp = msg.header.stamp
							pub.publish(pixel)

if __name__ == "__main__":
	rospy.init_node("input_process")
	
	# Publishes filtered pixels
	pub = rospy.Publisher('/filtered_pixels', Vector3Stamped, queue_size=10)
	
	# Publishes the time when the human reaches the object
	time_pub = rospy.Publisher('/human_time_topic', Time, queue_size=10)
	
	# Publishes the time of the pixel of the prediction 
	prediction_pixel_pub = rospy.Publisher('/prediction_pixel_topic', Time, queue_size=10)
	
	# Subscribes to the Openpose raw pixels
	sub = rospy.Subscriber('/openpose_ros/human_list', OpenPoseHumanList, openpose_callback)
	
	# Subscriber for reseting the game
	reset_sub = rospy.Subscriber('/reset_game_topic', Bool, reset_game_callback)

	s = rospy.Service('/next_motion', Empty, service)
	rospy.wait_for_service('/next_motion')

	# Check if we have received a signal for reseting the game
	while not rospy.is_shutdown():
		if reset_game:
			start, end, start_std, print_time = False, False, False, True
			x_prev, y_prev, time_prev, x_motion, y_motion, time_motion = [], [], [], [], [], []
			count = 0
			start_time = None
			prediction_pixel_time = None
			start_game = False
			reset_game = False