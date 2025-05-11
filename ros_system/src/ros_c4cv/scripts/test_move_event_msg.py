#!/usr/bin/env python3

import rospy
from ros_c4msgs.msg import BoardState, MoveMade
import random

prev_move = 1

def publisher():
    global prev_move
    rospy.init_node("move_made_pub", anonymous=True)
    pub = rospy.Publisher("c4/move_made", MoveMade, queue_size=10)

    while not rospy.is_shutdown():
        msg = MoveMade()
        state = BoardState()
        state.player1 = 1234123414
        state.player2 = 2134512345
        state.heights = (0, 7, 14, 21, 28, 35, 42)

        if prev_move == 2:
            msg.move_made = msg.PLAYER1_MOVE
            prev_move = 1
        else:
            msg.move_made = msg.PLAYER2_MOVE
            prev_move = 2
        msg.board_state = state

        input("Press enter to send message.")
        pub.publish(msg)
        rospy.loginfo("published message.")
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException as e:
        print(f"Interruption: {e}")

