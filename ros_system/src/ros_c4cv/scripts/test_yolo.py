#!/usr/bin/env python3

import rospy
from ros_c4msgs.msg import BoardState, MoveMade
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
from std_msgs.msg import String, Empty

from connect4_cv import StateValidator


class BoardStateDetectorYOLO:
    def __init__(self):
        rospy.init_node("board_state_detector_YOLO", anonymous=True)

        self.weights                = rospy.get_param("~weights", "/home/andy/work/ros_work/src/ros_c4cv/weights/april16.pt")
        self.confidence_thresh      = rospy.get_param("~confidence_threshold", 0.4)
        self.camera_topic           = rospy.get_param("~camera_topic", "/usb_cam/image_raw")
        self.board_state_topic      = rospy.get_param("~board_state_topic", "c4/board_state")
        self.detection_hz           = rospy.get_param("~detection_hz", 5)
        self.display_detections     = rospy.get_param("~display_detections", True)
        self.reset_topic            = "/c4/reset_all"
        
        try:
            self.model = YOLO(self.weights)
        except Exception as e:
            rospy.logerr(f"[BoardStateDetectorYOLO] error loading YOLO model:\n {e}")
            rospy.signal_shutdown()
            return
        self.class_names = self.model.names
        self.target_classes = ['Red Piece', 'Yellow Piece', 'No Piece']
        self.target_class_ids = [class_id for class_id, class_name in self.class_names.items() if class_name in self.target_classes]

        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_image_sub = rospy.Subscriber(
            name=self.camera_topic,
            data_class=Image,
            callback=self.image_cb,
            queue_size=1
        )
        self.reset_sub = rospy.Subscriber(
            name=self.reset_topic,
            data_class=Empty,
            callback=self.reset_state,
            queue_size=1
        )
        self.board_state_pub = rospy.Publisher(
            name=self.board_state_topic,
            data_class=BoardState,
            queue_size=10
        )
        self.board_cleared_pub = rospy.Publisher(
            name="/c4/board_cleared",
            data_class=String,
            queue_size=1
        )
        self.move_made_pub = rospy.Publisher(
            name="/c4/move_made",
            data_class=MoveMade,
            queue_size=1
        )
        self.detections_image_pub = rospy.Publisher(
            name="/c4/board_detections",
            data_class=Image,
            queue_size=1
        )
        self.detection_rate = rospy.Rate(self.detection_hz)

        self.reset_state(Empty())

        rospy.loginfo("[BoardStateDetectorYOLO] initialised successfully.")
        rospy.loginfo(f"Subscribed to {self.camera_topic} and publishing board state on {self.board_state_topic}.")

        while not rospy.is_shutdown():
            self.detect_board_state()

            if self.red_state is not None and self.yellow_state is not None:
                msg = BoardState()
                msg.player1 = self.red_state
                msg.player2 = self.yellow_state
                msg.heights = list(self.heights_state) if self.heights_state is not None else [0, 7, 14, 21, 28, 35, 42]
                self.board_state_pub.publish(msg)

            self.detection_rate.sleep()

    def reset_state(self, msg):
        self.board_state = None
        self.heights_state = None
        self.red_state = None
        self.yellow_state = None
        
        self.initial_board_state = 6*[7*["empty"]]
        self.validator = StateValidator(initial_state=self.initial_board_state)
        self.any_move_made = False

        self.previous_move = 2        

    def image_cb(self, msg):
        try:
            # rospy.loginfo("[BoardStateDetectorYOLO] Received new image.")
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # TODO check if we can get away with streaming the camera images as the 640x360 
            # instead of doing a scaling here
            # im_height, im_width, _ = latest_image.shape
            # self.latest_image = cv2.resize(self.latest_image, (640, 640))
        except CvBridgeError as e:
            rospy.logerr(e)

    def detect_board_state(self):
        if self.latest_image is not None:
            results = self.model(self.latest_image, verbose=False)
            if self.display_detections:
                self.display_bounding_boxes(results, self.latest_image)
            self.board_state_from_detections(results)
            if self.board_state == self.initial_board_state and self.any_move_made:
                temp_msg = String()
                temp_msg.data = "Board cleared."
                self.board_cleared_pub.publish(temp_msg)
                rospy.sleep(1)
                # rospy.signal_shutdown("Board was cleared. Shutting down.")
            if self.board_state is None:
                return
            self.board_state, change_needed = self.validator.get_valid_state(self.board_state)
            self.board_state_to_num(self.board_state)
            if change_needed:
                self.any_move_made = True
                move_made_msg = MoveMade()
                temp_state = BoardState()
                temp_state.player1 = self.red_state
                temp_state.player2 = self.yellow_state
                temp_state.heights = list(self.heights_state) if self.heights_state is not None else [0, 7, 14, 21, 28, 35, 42]
                if self.previous_move == 2:
                    move_made_msg.move_made = move_made_msg.PLAYER1_MOVE
                    self.previous_move = 1
                else:
                    move_made_msg.move_made = move_made_msg.PLAYER2_MOVE
                    self.previous_move = 2
                move_made_msg.board_state = temp_state
                self.move_made_pub.publish(move_made_msg)
                change_needed = False

    def board_state_from_detections(self, results):
        state = []

        # List of detections with (x, y) points (center coordinates of bounding boxes)
        detections = [((result[0]+result[2])//2, (result[1]+result[3])//2, result[4], result[5]) for result in results[0].boxes.data]
        filtered_detections = [d for d in detections if d[2] >= self.confidence_thresh]
        filtered_detections = [d for d in detections if int(d[3]) in self.target_class_ids]
        # Sort the detections by the y coordinate (ascending order)
        sorted_by_y = sorted(filtered_detections, key=lambda d: d[1])

        for row in range(6):
        # Get the top 7 detections with the smallest y (top row of the grid)
            top_row_detections = sorted_by_y[row*7+0:row*7+7]
            # print(top_row_detections)

            # Sort the top row detections by the x coordinate (ascending order)
            top_row_sorted_by_x = sorted(top_row_detections, key=lambda d: d[0])

            for x in top_row_sorted_by_x:
                name = self.class_names[int(x[3])]
                if name == 'No Piece':
                    state.append("empty")
                elif name == 'Yellow Piece':
                    state.append("yellow")
                elif name == 'Red Piece':
                    state.append("red")

        if len(state) == 42:
            self.board_state = []
            for row in range(6):
                temp = []
                for col in range(7):
                    temp.append(state[row*7 + col])
                self.board_state.append(temp)

    def display_bounding_boxes(self, results, image):
        for result in results[0].boxes.data:
            x1, y1, x2, y2, confidence, class_id = result
            
            if int(class_id) in self.target_class_ids and float(confidence) > self.confidence_thresh:
                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)
                
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 1)  # Green box with thickness 2
                
                name = self.class_names[int(class_id)]
                if name == 'No Piece':
                    name = "E"
                elif name == 'Yellow Piece':
                    name = "Y"
                elif name == 'Red Piece':
                    name = "R"
                label = f"{name}"
                
                (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(image, (x1, y1 - 20), (x1 + text_width, y1), (0, 0, 0), -1)
                cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        self.detections_image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        # cv2.imshow('Annotated Image', image)
        # cv2.waitKey(1)

    def board_state_to_num(self, board_state):
        self.red_state = 0
        self.yellow_state = 0
        if self.heights_state is None:
            self.heights_state = [0, 7, 14, 21, 28, 35, 42]

        heights_done = []
        for i, row in enumerate(board_state):
            for j, cell in enumerate(row):
                index = (5-i) + (j * 7)
                index_above = (5-i+1) + (j * 7) # just for the heights
                if cell == "red":
                    self.red_state |= (1 << index)
                    if j not in heights_done:
                        self.heights_state[j] = index_above
                        heights_done.append(j)
                elif cell == "yellow":
                    self.yellow_state |= (1 << index)
                    if j not in heights_done:
                        self.heights_state[j] = index_above
                        heights_done.append(j)
        self.heights_state.sort()
        if len(self.heights_state) != 7:
            rospy.logwarn("heights wrong")
            self.heights_state = [0, 7, 14, 21, 28, 35, 42]

if __name__ == "__main__":
    BoardStateDetectorYOLO()
