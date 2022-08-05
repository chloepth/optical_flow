#!/usr/bin/env python3
from colorsys import rgb_to_hls
import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import random
from statistics import mean


class OpticalFlowNode(object):
	'''A class to support Optical Flow using sonar imaging.
	'''

	def __init__(self):
		self.i= 0
		self.key_points=None
		self.prev_gray=None
		self.cur_points=None
		self.prev_frame=None
		self.mask=None
		self.br=None
		self.color_array=None


	def init_node(self, ns="~")->None:
		"""Init the node, fetch all paramaters from ROS.
		All parameters are explained in the yaml file.

		Args:
			ns (str, optional): The namespace of the node. Defaults to "~".
		"""
		self.window_leg = rospy.get_param(ns + "window_leg")
		maxLevel = rospy.get_param(ns + "maxLevel")
		termination = rospy.get_param(ns + "termination")
		quality = rospy.get_param(ns + "quality")
		flags = rospy.get_param(ns + "flags")

		self.lk_params = dict(winSize = (self.window_leg, self.window_leg),maxLevel=maxLevel,
										criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,termination, quality),
										flags=flags,minEigThreshold=1e-3)

		des_type = rospy.get_param(ns + "des_type")
		des_chan = rospy.get_param(ns + "des_chan")
		self.detector = cv2.AKAZE_create(descriptor_type= des_type,descriptor_channels=des_chan)

		self.bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True) # Function for Brute Force Feature Matcher

		self.sonar_sub = rospy.Subscriber('/sonar_oculus_node/image', Image, self.callback)

		self.br = CvBridge() # Initialize cv image convertor and AKAZE feature detector
		self.colors_array()

		rospy.loginfo("------------------------------")
		rospy.loginfo("OpticalFlowNode for USMMA data is initialized")


	def colors_array(self):
		""" Create array of random colors for drawing purposes.
		"""
		no_of_colors = 200
		self.color_array = np.empty((0,3), int) # Create empty array with 3 rows
		for n in range(no_of_colors):
		  color = [random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)]
		  self.color_array = np.append(self.color_array, np.array([color]), axis=0)


	def callback(self,data):
		"""Keypoints are defined every 30 frames.
		For the 'good' keypoints, optical flow method is applied.
		"""
		if (self.i%30==0):
			self.prev_frame = self.br.imgmsg_to_cv2(data) # Convert to openCV image
			self.prev_gray = cv2.cvtColor(self.prev_frame, cv2.COLOR_BGR2GRAY) # Convert to a grayscale image
			self.prev_gray = cv2.GaussianBlur(self.prev_gray, (11, 11), 10.0) # Use Gaussian Blur Filter

			self.key_points, descsi = self.detector.detectAndCompute(self.prev_gray, None) # Detect keypoints
			self.key_points = cv2.KeyPoint_convert(self.key_points) # Convert to coordinate tuples
			array_size = int(self.key_points.size)
			my_size = int(array_size/2)
			self.key_points.shape = (my_size, 1, 2)

			self.mask = np.zeros_like(self.prev_frame)
			self.i = self.i+1

		else:
			cur_frame = self.br.imgmsg_to_cv2(data)
			cur_gray = cv2.cvtColor(cur_frame, cv2.COLOR_BGR2GRAY)
			cur_gray = cv2.GaussianBlur(cur_gray, (11, 11), 10.0)

			# Calculate Optical Flow
			self.cur_points, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray,cur_gray,self.key_points, None,**self.lk_params)

			# Just resizing
			cur_size = int(self.cur_points.size)
			cur_size = int(cur_size/2)
			self.cur_points.shape = (cur_size, 1, 2)

			# Only use good points (had status 1)
			good_cur = self.cur_points[st == 1]
			good_prev = self.key_points[st == 1]

			# Make a loop to put points into an array
			for s, (cur, prev) in enumerate(zip(good_cur,good_prev)):
				a, b = cur.ravel()
				c, d = prev.ravel()

				# L1 : distance between new patch and original patch : error
				L1 = err[s]*self.window_leg
				if (L1 < 200):
					# Pick color number s from the array and turn its numbers into a list
					rand_color = self.color_array[s].tolist()
					# Draw a line on the self.mask
					self.mask = cv2.line(self.mask, (int(a), int(b)), (int(c), int(d)),rand_color, 2)
					frame = cv2.circle(cur_frame, (int(a), int(b)), 5,rand_color, -1)
					image = cv2.add(frame, self.mask)
					cv2.imshow('Processing Optical flow', image)
					cv2.waitKey(1)

			self.key_points = self.cur_points
			self.prev_gray = cur_gray
			self.i = self.i+1
