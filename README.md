# Optical Flow for BlueROV

These scripts are for tracking the feature transformation of an underwater fence viewed through sonar oculus M1200d.
It uses Lukas-Kanade method.

## Data

This code can be run with two differents data:
- *sample_data.bag* : data from USMMA. **Occulus M750d** imaging sonar is used for these data.
- *3_meter_orbit.bag* : data from the tank at STEVENS (the fence). **Occulus M1200** imaging sonar is used for these data.

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






