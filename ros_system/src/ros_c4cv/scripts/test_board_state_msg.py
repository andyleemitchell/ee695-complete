#!/usr/bin/env python3

import rospy
from ros_c4msgs.msg import BoardState

def publisher():
    rospy.init_node("board_state_pub", anonymous=True)
    pub = rospy.Publisher("c4/board_state", BoardState, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = BoardState()
        msg.player1 = 1234123414
        msg.player2 = 2134512345

        msg.heights = (0, 7, 14, 21, 28, 35, 42)

        pub.publish(msg)
        rospy.loginfo("published message.")
        rate.sleep()


if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException as e:
        print(f"Interruption: {e}")