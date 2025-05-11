#!/usr/bin/env python3

import rospy
from ros_c4msgs.msg import MoveMade, NextMove
from random import randint

# next_move_sent = False

def move_made_cb(msg):
    global next_move_sent

    if msg.move_made == msg.PLAYER1_MOVE:
        return

    next_move = NextMove()

    # simulated sleep for figuring out next move
    rospy.sleep(0.5)

    next_move.next_move = randint(0, 6)
    pub.publish(next_move)
    rospy.loginfo("Published next_move message.")
    # rospy.loginfo(f"Board state: {msg.board_state}")
    # next_move_sent = True


if __name__ == "__main__":
    rospy.init_node("next_move_pub", anonymous=True)
    pub = rospy.Publisher("c4/next_move", NextMove, queue_size=10)
    rospy.Subscriber(name="c4/move_made", data_class=MoveMade, callback=move_made_cb)
    while not rospy.is_shutdown():
        rospy.spin()
