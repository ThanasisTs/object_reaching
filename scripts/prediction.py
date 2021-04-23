#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped

import os
import scandir
import numpy as np
import pickle


models = {}
pixels = np.array([])
pub = None
count = 0

def callback(msg):
	global pub, models, count
	count += 1
	np.append(pixels, np.array([msg.point.x, msg.point.y]), axis=0)
	if pixels.size == 3:
		clf = pickle.load(open(models[count], 'rb'))
		prediction = clf.predict(pixels)
		pred_msg = Bool()
		pred_msg.data = True if prediction == 'L' else False
		pub.publish(pred_msg)

if __name__ == "__main__":
	rospy.init_node('prediction')

	os.chdir('/home/thanasis/catkin_ws/src/object_reaching/models')
	model_names = [i for i in scandir.scandir('svm')]
	model_names = sorted(model_names)

	for i in range(1, 11):
		models.update({i : model_names[i-1]})

	sub = rospy.Subscriber('/filtered_pixels', Vector3Stamped, callback)
	pub = rospy.Publisher('/prediction', Bool, queue_size=10)
	rospy.sleep(0.1)
	rospy.loginfo("Ready to accept pixels")
	rospy.spin()
