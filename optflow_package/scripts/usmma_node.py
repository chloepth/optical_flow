#!/usr/bin/env python
import rospy
import cv2

from optflow_package.usmma_optflow import OpticalFlowNode


if __name__ == "__main__":
    rospy.init_node("usmma", log_level=rospy.INFO)

    node = OpticalFlowNode()
    node.init_node()

    rospy.spin()
    cv2.destroyAllWindows()
