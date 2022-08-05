# Optical Flow for BlueROV

These scripts are for tracking the feature transformation of an underwater fence viewed through sonar oculus M1200d.
It uses Lukas-Kanade method.

## Data

This code can be run with two differents data:
- *sample_data.bag* : data from USMMA. **Occulus M750d** imaging sonar is used for these data.
- *3_meter_orbit.bag* : data from the tank at STEVENS (the fence). **Occulus M1200** imaging sonar is used for these data.

## Differences between the fence and the USMMA data

Each data has its own python file because of three main differences:

- **Subscriber**  
*fence_optflow.py* subscribe to */sonar_oculus_node/M1200d/image*  
*usmma_optflow.py* subscribe to */sonar_oculus_node/image*
- **Cropping function**  
The data from the tank (the fence) needs to be crop in order to be workable with optical flow: function *get_image()* in *fence_optflow.py*
- **Parameters**  
The Lukas-Kanade parameters are not the same. See the two differents yaml files.

## How to run

**For the fence data:**

- cd ~/ws
- source devel/setup.bash
- roslaunch optflow_package fence.launch

- rosbag play 3_meter_orbit.bag 

**For the USMMA data:**

- cd ~/ws
- source devel/setup.bash
- roslaunch optflow_package usmma.launch
- rosrun sonar_oculus oculus_viewer_2.py 
- rosbag play sample_data.bag 

*Suggestion* : as the data bags are quite long, you could start the bag file at 100" with this command:
- rosbag play -s100 sample_data.bag 

## Dependencies
**Python**
- cv_bridge
- cv2
- numpy
- rosbag
- sensor_msgs
- random
- pyyaml

**ROS**
- ROS-noetic
- catkin_ws

*Note: Use catkin build instead of catkin make for the workspace*

## Functions description

- For both python file (*fence_optflow.py* and *usmma_optflow.py*) : 

__init__() : Define all class parameters.  
**init_node()** : Init the node, fetch all paramaters from ROS. All parameters are explained in the yaml file.   
**colors_array()** : Create array of random colors for drawing purposes.  
**callback()** : Callback function is dealing with Optical Flow. Keypoints are defined every 30 frames.

- One more function for *fence_optflow.py*:  

**get_image()** : This function is used to crop the image from the fence.







