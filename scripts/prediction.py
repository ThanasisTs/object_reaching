#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Time
from geometry_msgs.msg import Vector3Stamped

import os
import sys
from scipy.spatial import distance
import numpy as np
import pickle
from sklearn.linear_model import LogisticRegression

models = {}
pixels = np.array([])
pub = None
count = 0
prediction_flag = False

# Reset the game. Initialize corresponding variables
def reset_game_callback(msg):
	global prediction_flag
	rospy.logwarn('Reset the prediction module. Waiting for signal to record human motion...')
	prediction_flag = msg.data

# Callback for predicting based on pixels
def callback(msg):
	global pub, models, count, pixels

	count += 1
	if count == 1:
		pixels = np.append(pixels, np.array([msg.vector.x, msg.vector.y]))
		pixels = pixels.reshape(1, -1)
	else:
		pixels = np.append(pixels, np.array([msg.vector.x, msg.vector.y]).reshape(1, -1), axis=0)

	if pixels.size == 8: # {3 pixels : 6, 4 pixels : 8, 5 pixels: 10, 6 pixels : 12}
		clf = pickle.load(open(models[count], 'rb'))
		pixels_reshaped = pixels.reshape(1, -1)
		prediction = clf.predict(pixels_reshaped)
		pred_msg = Bool()
		pred_msg.data = True if prediction == 'R' else False
		rospy.loginfo('Predicted {} '.format(prediction))
		pub.publish(pred_msg)
		prediction_time = Time()
		prediction_time.data = rospy.Time.now()
		prediction_time_pub.publish(prediction_time)
		

# Callback for predicting based on distances
def callback_dis(msg):
	global pub, models, count, pixels, obj_R, obj_L

	count += 1
	if count == 1:
		pixels = np.append(pixels, np.array([msg.vector.x, msg.vector.y]))
		pixels = pixels.reshape(1, -1)
	else:
		pixels = np.append(pixels, np.array([msg.vector.x, msg.vector.y]).reshape(1, -1), axis=0)

	if pixels.size == 8: # {3 pixels : 6, 4 pixels : 8, 5 pixels: 10, 6 pixels : 12}
		clf = pickle.load(open(models[count], 'rb'))		
		prediction = clf.predict(np.array([[distance.euclidean([msg.vector.x, msg.vector.y], obj_R), distance.euclidean([msg.vector.x, msg.vector.y], obj_L)]]))
		pred_msg = Bool()
		pred_msg.data = True if prediction == 1 else False
		rospy.loginfo('Predicted {} '.format(prediction))
		pub.publish(pred_msg)
		prediction_time = Time()
		prediction_time.data = rospy.Time.now()
		prediction_time_pub.publish(prediction_time)

if __name__ == "__main__":
	rospy.init_node('prediction')

	# Load prediction models
	model_names = [i for i in os.listdir(sys.argv[1])]
	model_names = sorted(model_names)
	
	obj_R = rospy.get_param("prediction/obj_R", [None, None])
	obj_L = rospy.get_param("prediction/obj_L", [None, None])
	
	for i in range(1, 11):
		models.update({i : model_names[i-1]})

	# Subscribe to the filtered pixels
	sub = rospy.Subscriber('/filtered_pixels', Vector3Stamped, callback_dis)
	
	# Subcribe for reseting the game 
	reset_sub = rospy.Subscriber('/reset_game_topic', Bool, reset_game_callback)

	# Publishes the output of the prediction
	pub = rospy.Publisher('/prediction', Bool, queue_size=10)
	
	# Publishes the time of the prediction
	prediction_time_pub = rospy.Publisher('/prediction_time_topic', Time, queue_size=10)
	
	# Check if we have received a signal for reseting the game
	while not rospy.is_shutdown():
		if prediction_flag:
			pixels = np.array([])
			count = 0
			prediction_flag = False
