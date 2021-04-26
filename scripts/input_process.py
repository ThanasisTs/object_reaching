#!/usr/bin/env python
import rospy
from openpose_ros_msgs.msg import OpenPoseHumanList
from geometry_msgs.msg import Vector3Stamped

import numpy as np


start, end, start_std = False, False, False
pub = None
x_prev, y_prev, time_prev, x_motion, y_motion, time_motion = [], [], [], [], [], []
count = 0

# Removal of NaNs, outliers and redundant points at the start of the motion
# (works in real time and in offline mode)
def openpose_callback(msg):
	global pub, start, end, start_std, count, x_prev, y_prev, time_prev
	x = msg.human_list[0].body_key_points_with_prob[4].x
	y = msg.human_list[0].body_key_points_with_prob[4].y
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
						x_prev, y_prev, time_prev = [], [], []
						start = True
					if start:
						if std_total > 10:
							end = True
						if std_total <= 10 and end:
							# print("Motion ended at sample {}".format(count))
							return
						x_motion.append(x)
						y_motion.append(y)
						time_motion.append(time)
						pixel = Vector3Stamped()
						pixel.vector.x = x
						pixel.vector.y = y
						pixel.header.stamp = msg.header.stamp
						pub.publish(pixel)

if __name__ == "__main__":
	rospy.init_node("input_process")
	pub = rospy.Publisher('/filtered_pixels', Vector3Stamped, queue_size=10)
	sub = rospy.Subscriber('/openpose_ros/human_list', OpenPoseHumanList, openpose_callback)
	rospy.loginfo("Ready to record human movement")
	rospy.spin()