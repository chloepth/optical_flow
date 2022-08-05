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
import yaml

class OpticalFlowNode(object):
    '''A class to support Optical Flow using Sonar imaging
    '''

    def __init__(self):
        self.i= 0
        self.prev_gray=None
        self.key_points=None
        self.draw_mask=None
        self.color_array=None
        self.prev_frame=None
        self.cur_points=None
        self.first=True


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
        minEigThreshold = rospy.get_param(ns + "minEigThreshold")

        self.lk_params = dict(winSize = (self.window_leg, self.window_leg),maxLevel=maxLevel,
                                        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,termination, quality),
                                        flags=flags,minEigThreshold=1e-2)

        des_type = rospy.get_param(ns + "des_type")
        des_chan = rospy.get_param(ns + "des_chan")
        self.detector = cv2.AKAZE_create(descriptor_type= des_type,descriptor_channels=des_chan)

        # Function for Brute Force Feature Matcher
        self.bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)

        self.sonar_sub = rospy.Subscriber('/sonar_oculus_node/M1200d/image', Image, self.callback)
        self.colors_array()
        self.br = CvBridge() # Initialize cv image convertor and AKAZE feature detector

        rospy.loginfo("------------------------------")
        rospy.loginfo("OpticalFlowNode for Fence data is initialized")


    def get_image(self,data):
        """Crop the image in order to just have the fence.
        Args: data from the M1200d
        Returns: the cropped image in color and in grayscale

        """
        frame = self.br.imgmsg_to_cv2(data)  # Convert the frame's ROS image data to an opencv image
        points = np.array([[425, 400], [550, 375], [675, 400],[675, 470], [420, 470]]) # Pentagon cropped to fence position
        points = points.reshape((-1, 1, 2))
        mask = np.zeros(frame.shape[0:2], dtype=np.uint8)  # Create a mask
        shape = cv2.drawContours(mask, [points], -1, (255, 255, 255), -1, cv2.LINE_AA)   # Draw and fill the smooth polygon

        res = cv2.bitwise_and(frame, frame, mask = mask)       # Define the region we need
        rect = cv2.boundingRect(points) # Returns (x,y,w,h) of the rectangle around the polygon
        cropped_small = res[rect[1]: rect[1] + rect[3], rect[0]: rect[0] + rect[2]]

        # Resize the cropped image so we can see it big
        scale_percent = 300
        scale_factor = 3
        width = int(cropped_small.shape[1] * scale_percent / 100)
        height = int(cropped_small.shape[0] * scale_percent / 100)
        cropped = cv2.resize(cropped_small, [width, height])

        color_frame = cropped
        gray_frame = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        return color_frame, gray_frame


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
        if (self.i%50==0):
            self.prev_frame, self.prev_gray = self.get_image(data)

            self.key_points, descsi = self.detector.detectAndCompute(self.prev_gray, None) # AKAZE detect keypoints from the first frame
            self.key_points = cv2.KeyPoint_convert(self.key_points) # Convert key points to coordinate tuples

            if self.key_points != ():
                array_size = int(self.key_points.size)
                my_size = int(array_size/2)
                self.key_points.shape = (my_size, 1, 2)

                self.mask = np.zeros_like(self.prev_frame)
                self.i = self.i+1
            else:
                if self.first:
                    print('_____________________','\n'
                    'Im looking for keypoints...','\n',
                    'This can take a while if youre running the bag at 0:')
                    self.first=False

        else:
            cur_frame, cur_gray = self.get_image(data)

            # Calculate optical flow
            self.cur_points, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray,cur_gray,self.key_points, None,**self.lk_params)

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

                #  L1 distance between new patch and original patch : error
                L1 = err[s]*self.window_leg

                if  L1 < 100:
                    rand_color = self.color_array[s].tolist()
                    mask_line = cv2.line(self.mask, (int(a), int(b)), (int(c), int(d)),
                                    rand_color, 2)
                    frame = cv2.circle(cur_frame, (int(a), int(b)), 5,
                                   rand_color, -1)
                    image = cv2.add(mask_line, frame)
                    cv2.imshow('Processing Optical Flow', image)
                    cv2.waitKey(1)

            self.key_points = self.cur_points
            self.prev_gray = cur_gray
            self.i = self.i+1
