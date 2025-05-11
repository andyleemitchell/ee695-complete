#!/usr/bin/env python3

import rospy
from ros_c4msgs.msg import MoveMade, NextMove, BoardState
from ros_c4msgs.srv import GetNextMove

class NextMoveWrapper:
    def __init__(self):
        rospy.init_node("next_move_pub", anonymous=True)
        self.mcts_pub = rospy.Publisher("/c4/wrapper_mcts", MoveMade, queue_size=10)
        # rospy.Subscriber(name="/c4/internal_next_move", data_class=MoveMade, callback=self.mcts_move)
        self.service = rospy.Service(
            name="/c4/get_next_move",
            service_class=GetNextMove,
            handler=self.get_next_move
        )
        while not rospy.is_shutdown():
            rospy.spin()

    def get_next_move(self, request):
        rospy.loginfo("[NextMoveWrapper] received request for next move.")
        
        mcts_req = MoveMade()
        mcts_req.board_state = request.move_made.board_state
        mcts_req.move_made = mcts_req.PLAYER2_MOVE

        self.mcts_pub.publish(mcts_req)
        mcts_resp = rospy.wait_for_message(topic="/c4/internal_next_move", topic_type=NextMove)
        rospy.loginfo("[NextMoveWrapper] sending response.")
        return mcts_resp

if __name__ == "__main__":
    NextMoveWrapper()
