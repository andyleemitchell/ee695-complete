#!/usr/bin/env python3

import connect4_cv.state_validator
import numpy as np
import rospy
import cv2
from ros_c4msgs.msg import BoardState, MoveMade
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
# detector = connect4_cv.ConnectFourStateDetector()
# validator = connect4_cv.state_validator.StateValidator()

latest_image = None
valid_board_state = None
red_state = None
yellow_state = None
heights_state = None
board_state_global = None # this won't be the validated state

change_needed = False
prev_move = 1

def close_gaps_and_refine_circles(mask):
    # 1. Input Validation (Important!)
    if mask is None:
        raise ValueError("Input mask cannot be None.")
    if mask.ndim != 2:
        raise ValueError("Input mask must be a 2D grayscale image.")
    if mask.dtype != np.uint8:
        # Convert to uint8 if not already.  Handle potential clipping/overflow.
        mask = np.clip(mask, 0, 255).astype(np.uint8)
        # Alternatively, you could raise an error:
        # raise ValueError("Input mask must be of type np.uint8.")

    # 2. Initial Closing (Morphological)
    kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_small)

    # 3.  Find Contours (after initial closing)
    contours, _ = cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 4. Create a new mask for refined circles
    refined_mask = np.zeros_like(mask)

    # 5. Iterate through contours and fit circles
    for contour in contours:
        if len(contour) >= 5:
            (x, y), (MA, ma), angle = cv2.fitEllipse(contour)
            aspect_ratio = MA / ma if ma != 0 else 0
            if 0.7 < aspect_ratio < 1.3:
                radius = int((MA + ma) / 4)
                cv2.circle(refined_mask, (int(x), int(y)), radius, 255, -1)
            else:
                cv2.drawContours(refined_mask, [contour], 0, 255, -1)
        else:
            cv2.drawContours(refined_mask, [contour], 0, 255, -1)

    # 6. Final Closing (Optional)
    kernel_final = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    final_mask = cv2.morphologyEx(refined_mask, cv2.MORPH_CLOSE, kernel_final)

    return final_mask

def image_cb(msg):
    global latest_image
    try:
        latest_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # rospy.loginfo("Received new image.")
    except CvBridgeError as e:
        rospy.logerr(e)

def detection(frame):
    global valid_board_state, change_needed, heights_state, board_state_global
    rospy.loginfo("Doing detection.")
    # frame = cv2.bilateralFilter(frame, 15, 190, 190)
    # small_frame = cv2.resize(frame, (frame.shape[1] // 4, frame.shape[0] // 4))
    # small_frame = cv2.bilateralFilter(small_frame, 15, 200, 200)
    # small_frame = cv2.pyrMeanShiftFiltering(small_frame, sp=20, sr=20)
    # frame = cv2.resize(small_frame, (frame.shape[1], frame.shape[0]))
    # cv2.imshow("blurred", frame)
    # cv2.waitKey(1)
    # small_frame = cv2.resize(frame, (frame.shape[1] // 4, frame.shape[0] // 4))
    # # small_frame = cv2.bilateralFilter(small_frame, 15, 200, 200)

    # small_frame = cv2.pyrMeanShiftFiltering(small_frame, sp=20, sr=40)
    # small_frame = cv2.bilateralFilter(small_frame, d=15, sigmaColor=100, sigmaSpace=100)

    # frame = cv2.resize(small_frame, (frame.shape[1], frame.shape[0]))
    board_state = detector.detect(frame=frame)
    cv2.imshow("red mask", detector.red_mask)
    cv2.imshow("yellow mask", detector.yellow_mask)
    # rospy.logwarn(detector.red_thresh)
    # rospy.logwarn(detector.yellow_thresh)
    # cv2.imshow("frame", frame)
    cv2.waitKey(1)
    # rospy.logerr(50*"-")
    for row in board_state:
        rospy.logerr(row)
    # rospy.logerr(50*"-")
    rospy.loginfo("Doing validation.")
    board_state, change_needed_local = validator.get_valid_state(board_state=board_state)
    # rospy.logerr(50*"-")
    for row in board_state:
        rospy.logwarn(row)
    # rospy.logerr(50*"-")
    if change_needed_local:
        rospy.loginfo("New state detected.")
        valid_board_state = board_state
        # heights_state = [0, 7, 14, 21, 28, 35, 42]
        change_needed = True
    board_state_global = board_state

def board_state_to_num(board_state):
    global red_state, yellow_state, heights_state
    rospy.logerr("Running board_state_to_num")
    red_state = 0
    yellow_state = 0
    if heights_state is None:
        heights_state = [0, 7, 14, 21, 28, 35, 42]
    # heights_state.clear()

    heights_done = []
    for i, row in enumerate(board_state):
        for j, cell in enumerate(row):
            index = (5-i) + (j * 7)
            index_above = (5-i+1) + (j * 7) # just for the heights
            if cell == "red":
                red_state |= (1 << index)
                if j not in heights_done:
                    heights_state[j] = index_above
                    heights_done.append(j)
            elif cell == "yellow":
                yellow_state |= (1 << index)
                if j not in heights_done:
                    heights_state[j] = index_above
                    heights_done.append(j)
    rospy.logerr(heights_state)
    heights_state.sort()
    if len(heights_state) != 7:
        heights_state = [0, 7, 14, 21, 28, 35, 42]
    # if heights_state is not None else [0, 7, 14, 21, 28, 35, 42]
    rospy.logerr(heights_state)


def publisher():
    global change_needed, prev_move, valid_board_state, red_state, yellow_state, heights_state, board_state_global
    rospy.Subscriber("/usb_cam/image_raw", Image, callback=image_cb, queue_size=1)
    pub = rospy.Publisher("c4/board_state", BoardState, queue_size=10)
    event_pub = rospy.Publisher("c4/move_made", MoveMade, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if latest_image is not None:
            detection(frame=latest_image)
        # if valid_board_state is not None:
        #     board_state_to_num()
        #     rospy.logwarn(f"red state: {red_state}")
        #     rospy.logwarn(f"yellow state: {yellow_state}")
        if board_state_global is not None:
            board_state_to_num(board_state_global)
            msg = BoardState()
            msg.player1 = red_state
            msg.player2 = yellow_state
            msg.heights = list(heights_state) if heights_state is not None else [0, 7, 14, 21, 28, 35, 42]

            pub.publish(msg)
            rospy.loginfo("published board state message.")

        if change_needed:
            board_state_to_num(valid_board_state)
            msg = MoveMade()
            state = BoardState()
            # TODO these will change depending on who moves first!!!
            state.player1 = red_state
            state.player2 = yellow_state
            rospy.logerr(heights_state)
            state.heights = list(heights_state)
            if prev_move == 2:
                msg.move_made = msg.PLAYER1_MOVE
                prev_move = 1
            else:
                msg.move_made = msg.PLAYER2_MOVE
                prev_move = 2
            msg.board_state = state
            event_pub.publish(msg)
            rospy.logerr("published move made message.")
            # for row in valid_board_state:
            #     rospy.logwarn(row)
            change_needed = False

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("board_state_pub", anonymous=True)
    json_file = rospy.get_param("~json_file")
    config_file = rospy.get_param("~config_file")
    rospy.logwarn(json_file)
    rospy.logwarn(config_file)

    detector = connect4_cv.ConnectFourStateDetector(conf_file=config_file, noise_lim=700)
    validator = connect4_cv.state_validator.StateValidator(json_file=json_file)

    rospy.loginfo("Successfully loaded detector and validator.")
    try:
        publisher()
    except rospy.ROSInterruptException as e:
        print(f"Interruption: {e}")

