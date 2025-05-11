#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image

class CounterLocator:
    def __init__(self):
        rospy.init_node("counter_locator", anonymous=True)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.latest_depth_image = None
        rospy.spin()

    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.medianBlur(frame, 5)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])

        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in red_contours:
            if cv2.contourArea(contour) > 5000:
                x, y, w, h = cv2.boundingRect(contour)
                centre_x, centre_y = x+w // 2, y+h // 2

                if self.latest_depth_image is not None:
                    depth_value = self.latest_depth_image[centre_y, centre_x]
                    # rospy.loginfo(f"Red at [{centre_x}, {centre_y}], depth: {depth_value}mm")
                
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,255), 2)
                cv2.putText(frame, f"Red: {depth_value}mm", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

        for contour in yellow_contours:
            if cv2.contourArea(contour) > 5000:
                x, y, w, h = cv2.boundingRect(contour)
                centre_x, centre_y = x+w // 2, y+h // 2

                if self.latest_depth_image is not None:
                    depth_value = self.latest_depth_image[centre_y, centre_x]
                    # rospy.loginfo(f"Yellow at [{centre_x}, {centre_y}], depth: {depth_value}mm")
                
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,255), 2)
                cv2.putText(frame, f"Yellow: {depth_value}mm", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

        # other stuff
        mask = red_mask.copy()
        kernel = np.ones((25,25), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ellipse_data = []
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if contour_area < 1000:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            if not (0.8 <= aspect_ratio <= 1.2):
                continue
            if len(contour) >= 5:
                ellipse = cv2.fitEllipse(contour)
                center, axes, angle = ellipse
                cv2.ellipse(frame, ellipse, (0, 255, 0), 2)

                ellipse_info = {
                    'center': center,
                    'axes': axes,
                    'angle': angle,
                    'contour_area': contour_area,
                    'aspect_ratio': aspect_ratio,
                    'is_valid': True
                }
                ellipse_data.append(ellipse_info)
        largest_ellipse = None
        for ellipse in ellipse_data:
            if largest_ellipse is None:
                largest_ellipse = ellipse
            if ellipse["contour_area"] > largest_ellipse["contour_area"]:
                largest_ellipse = ellipse

        # print(largest_ellipse["center"]) if largest_ellipse is not None else print()

        if largest_ellipse is not None:
            centre_x = int(largest_ellipse["center"][0])
            centre_y = int(largest_ellipse["center"][1])
            depth = self.latest_depth_image[centre_y, centre_x]
            print(f"Largest counter is {depth}mm away.")

        cv2.imshow("Detected objects", frame)
        cv2.imshow("red mask", mask)
        # cv2.imshow("circles", red_mask)
        cv2.waitKey(1)

if __name__ == "__main__":
    CounterLocator()
