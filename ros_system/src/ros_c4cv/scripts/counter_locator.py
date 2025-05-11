#!/usr/bin/env python3

import rospy
# import tf
import tf2_ros
import tf2_geometry_msgs # Required for PointStamped transformation
from sensor_msgs.msg import Image, CameraInfo
import geometry_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

from ros_c4msgs.srv import GetCounterLocation, GetCounterLocationResponse


class CounterLocator:
    def __init__(self):
        rospy.init_node("counter_locator", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        
        self.service = rospy.Service("/c4/get_counter_location", GetCounterLocation, self.response_point)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        pub = rospy.Publisher("/testing/counter_location", geometry_msgs.msg.Point, queue_size=1)
        self.image_pub = rospy.Publisher("/testing/detected_counter", Image, queue_size=1)

        self.latest_depth_image = None
        self.get_camera_info()

        self.point = None
        self.counter_found = False
        self.rate = rospy.Rate(1)
        rospy.spin()
        # while not rospy.is_shutdown():
        #     if self.point is not None:
        #         pub.publish(self.point)
        #         rospy.loginfo("[CounterLocator] published message.")
        #     self.rate.sleep()

    def response_point(self, request):
        rospy.loginfo("[get_counter_location] got request")
        self.do_location(request)
        response = GetCounterLocationResponse()
        response.point = self.point
        response.counter_found = self.counter_found
        rospy.loginfo("[get_counter_location] sending response")
        return response

    def get_camera_info(self):
        self.camera_intrinsics = {}
        self.camera_info = rospy.wait_for_message("/camera/aligned_depth_to_color/camera_info", CameraInfo, timeout=10.0)
        if self.camera_info:
            self.camera_optical_frame = self.camera_info.header.frame_id
            self.camera_intrinsics['fx'] = self.camera_info.K[0]
            self.camera_intrinsics['fy'] = self.camera_info.K[4]
            self.camera_intrinsics['cx'] = self.camera_info.K[2]
            self.camera_intrinsics['cy'] = self.camera_info.K[5]
            # rospy.loginfo("Received camera info. ",
            #               f"Optical frame: '{self.camera_optical_frame}'. ",
            #               f"Intrinsics (fx,fy,cx,cy): ({self.camera_intrinsics['fx']}, {self.camera_intrinsics['fy']}, {self.camera_intrinsics['cx']}, {self.camera_intrinsics['cy']})."
            # )
            rospy.logwarn(self.camera_intrinsics)
        else:
            rospy.logerr("Failed to receive camera info.")
            rospy.signal_shutdown("No camera info received.")
            return

    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def do_location(self, request):
        # this should be more intelligent (how to decide between two etc.)
        # right now it just returns the largest counter it can see
        x, y, depth = self.locate_counter(request.colour)
        # convert to camera frame coords

        if x > 0 and y > 0 and depth > 0:
            # print(f"Image: [{x}, {y}]")
            cam_x, cam_y, cam_z = self.pixel_to_camera_frame(x, y, depth)
            # print(f"Camera: [{cam_x}, {cam_y}, {cam_z}]")

            camera_point = geometry_msgs.msg.PointStamped()
            camera_point.header.frame_id = "camera_color_optical_frame"
            camera_point.header.stamp = rospy.Time(0)
            camera_point.point.x = cam_x
            camera_point.point.y = cam_y
            camera_point.point.z = cam_z + 0.015

            world_frame_point = self.tf_buffer.transform(
                camera_point,
                "world",
                rospy.Duration(0.1)
            )

            # print(f"World: [{world_frame_point.point.x}, {world_frame_point.point.y}, {world_frame_point.point.z}]")
            self.point = geometry_msgs.msg.Point(
                world_frame_point.point.x,
                world_frame_point.point.y,
                world_frame_point.point.z
            )
            self.counter_found = True
            # print("point found...")

        # else:
        #     self.point = geometry_msgs.msg.Point(
        #         0,0,0
        #     )
        #     self.counter_found = False


    def locate_counter(self, colour):
        self.frame = cv2.medianBlur(self.frame, 5)

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])

        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        if colour == "red":
            mask = red_mask.copy()
        elif colour == "yellow":
            mask = yellow_mask.copy()
        else:
            return
        
        ellipse_data = []
        kernel = np.ones((25,25), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
                cv2.ellipse(self.frame, ellipse, (0, 255, 0), 2)

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

        centre_x = None
        centre_y = None
        depth = None
        if largest_ellipse is not None:
            centre_x = int(largest_ellipse["center"][0])
            centre_y = int(largest_ellipse["center"][1])
            depth = self.latest_depth_image[centre_y, centre_x]
            # print(depth)
            # print(f"Largest counter is {depth}mm away at [{centre_x}, {centre_y}].")

        # cv2.imshow("Detected objects", self.frame)
        # cv2.imshow("mask mask", mask)
        # cv2.waitKey(1)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8"))

        if centre_x is not None and centre_y is not None and depth is not None:
            return centre_x, centre_y, depth
        else:
            return -1, -1, -1

    def pixel_to_camera_frame(self, x, y, depth):
        # need to convert depth to metres
        depth_m = depth / 1000.0
        X = (x - self.camera_intrinsics["cx"]) * depth_m / self.camera_intrinsics["fx"]
        Y = (y - self.camera_intrinsics["cy"]) * depth_m / self.camera_intrinsics["fy"]
        Z = depth_m
        return X, Y, Z

if __name__ == "__main__":
    CounterLocator()
