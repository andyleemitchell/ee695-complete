#!/usr/bin/env python3

import json
import os
import connect4_cv.state_validator
import numpy as np
import rospy
import cv2
from ros_c4msgs.msg import BoardState, MoveMade
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
detector = connect4_cv.ConnectFourStateDetector()

json_file = ".board_state.json"
temp_json = json_file + ".tmp"

if __name__ == "__main__":
    rospy.init_node('calibrate_from_usb_cam', anonymous=True)

    # Wait for a single message from the camera
    rospy.loginfo("Waiting for camera image...")
    image_msg = rospy.wait_for_message("/usb_cam/image_raw", Image)
    rospy.loginfo("Image received.")

    try:
        # Convert ROS image to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        # Display the image
        cv2.imshow("Calibration Image", cv_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Run detection on the image
        board_state = detector.detect(frame=cv_image)
        json_state = {"board_state": board_state}

        with open(temp_json, mode="w", encoding="utf-8") as write_file:
            json.dump(json_state, write_file, indent=4)

        os.replace(temp_json, json_file)
        for row in board_state:
            print(row)

        rospy.loginfo("saving detection........")
        rospy.sleep(1)
        rospy.signal_shutdown("Calibration complete.")

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
