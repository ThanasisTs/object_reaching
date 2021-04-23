#!/usr/bin/env python
import rospy
from openpose_ros_msgs.msg import OpenPoseHumanList
from geometry_msgs.msg import Vector3Stamped

import numpy as np


start, first_point = False, False
pub = None
x_prev, y_prev, x_motion, y_motion = [], [], [], []
count = 0

# Removal of NaNs, outliers and redundant points at the start of the motion
# (works in real time and in offline mode)
def openpose_callback(msg):
	global pub, start, first_point, count, x_prev, y_prev
	x = msg.human_list[0].body_keypoints_with_prob[4].x
	y = msg.human_list[0].body_keypoints_with_prob[4].y
	
	if x != 0 and y != 0:
		if not first_point:
			pixel = Vector3Stamped()
			pixel.vector.x = x
			pixel.vector.y = y
			pixel.header.stamp = msg.header.stamp
			pub.publise(pixel)
			x_prev.append(x)
			y_prev.append(y)
			count += 1
			first_point = True
		else:
			if abs(x-x_prev[-1]) < 30 and abs(y-y_prev[-1]) < 30:
				if len(x_prev) < 10:
					x_prev.append(x)
					y_prev.append(y)
					count += 1
					return
				elif len(x_prev) == 10:
					count += 1
					del x_prev[0], y_prev[0]
					x_prev.append(x)
					y_prev.append(y)
					std_total = np.sqrt(np.std(x_prev)**2 + np.std(y_prev)**2)
					if (not start) and std_total > 1.5:
						rospy.logwarn("Motion started at sample {}".format(count))
						start = True
					if start:
						x_motion.append(x)
						y_motion.append(y)
						if len(x_motion) == 6:
							del x_motion[0], y_motion[0]
							std_total = np.sqrt(np.std(x_motion)**2 + np.std(y_motion))
							if std_total > 5:
								rospy.logwarn("Motion ended at sample {}".format(count))
								return
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