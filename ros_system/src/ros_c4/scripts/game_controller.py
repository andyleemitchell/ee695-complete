#!/usr/bin/env python3
import rospy
from ros_c4msgs.msg import MoveMade, NextMove, BoardState
from ros_c4msgs.srv import GetNextMove, GetNextMoveRequest, PlaceCounter, PlaceCounterRequest
from std_msgs.msg import Empty

class C4GameController:
    """
    Main controller for connect four games. 
    Implements an FSM to handle game actions etc.
    """

    def __init__(self):
        rospy.init_node("c4_game_controller", anonymous=True)

        # subscribers
        self.board_state_sub =rospy.Subscriber(
            name="/c4/board_state",
            data_class=BoardState,
            callback=self.board_state_cb
        )
        self.reset_sub = rospy.Subscriber(
            name="/c4/reset_all",
            data_class=Empty,
            callback=self.reset_state,
            queue_size=1
        )
        self.move_made_sub = rospy.Subscriber(
            name="/c4/move_made",
            data_class=MoveMade,
            callback=self.move_made_check,
            queue_size=1
        )


        # publishers
        # service proxies
        rospy.wait_for_service(service="/c4/get_next_move")
        rospy.wait_for_service(service="/c4/place_counter")
        self.next_move_srv = rospy.ServiceProxy(
            name="/c4/get_next_move",
            service_class=GetNextMove
        )
        self.place_counter_srv = rospy.ServiceProxy(
            name="/c4/place_counter",
            service_class=PlaceCounter
        )

        self.order = "robot_first"
        self.robot_colour = "red"
        self.difficulty = 1

        self.game_state = "playing"

        rospy.loginfo("[C4GameController] successfully initialised.")

    def setup_game(self):
        while True:
            order = input("Who goes first: [H]uman or [R]obot: ")
            if order in ["H", "h", "R", "r"]:
                if order in ["H", "h"]:
                    self.order = "human_first"
                else:
                    self.order = "robot_first"
                break
        while True:
            colour = input("What colour is the robot using: [R]ed or [Y]ellow: ")
            if colour in ["Y", "y", "R", "r"]:
                if colour in ["R", "r"]:
                    self.robot_colour = "red"
                else:
                    self.robot_colour = "yellow"
                break
        while True:
            difficulty = int(input("Select difficulty 1-3: "))
            if 1 <= difficulty <= 3:
                self.difficulty = difficulty
                break

        # set options as params
        rospy.set_param(
            param_name="/c4/order",
            param_value=self.order
        )
        rospy.set_param(
            param_name="/c4/robot_colour",
            param_value=self.robot_colour
        )
        rospy.set_param(
            param_name="/c4/difficulty",
            param_value=self.difficulty
        )

    def run(self):
        # do first place if we have to
        if self.order == "robot_first":
            rospy.loginfo("[C4GameController] doing first move.")
            move_made = MoveMade()
            move_made.board_state.player1 = 0
            move_made.board_state.player2 = 0
            move_made.board_state.heights = [0, 7, 14, 21, 28, 35, 42]
            move_made.move_made = move_made.PLAYER2_MOVE
            rospy.loginfo("[C4GameController] requesting next move")
            next_move_msg = self.next_move_srv(move_made)
            # pass next_move to counter placer
            rospy.loginfo("[C4GameController] performing next move")
            place_req = PlaceCounterRequest()
            place_req.next_move.next_move = next_move_msg.next_move.next_move
            success = self.place_counter_srv(place_req)
            if success:
                rospy.loginfo("[C4GameController] successfully placed counter.")

        while not rospy.is_shutdown():
            rospy.loginfo("[C4GameController] waiting for move made message")
            move_made = rospy.wait_for_message(
                topic="/c4/move_made",
                topic_type=MoveMade
            )
            if self.order == "robot_first":
                if move_made.move_made == move_made.PLAYER1_MOVE:
                    rospy.loginfo("[C4GameController] detected the robot's move. Doing nothing.")
                    continue
                else:
                    rospy.loginfo("[C4GameController] detected the opponents's move. Making a move.")
            else:
                if move_made.move_made == move_made.PLAYER2_MOVE:
                    rospy.loginfo("[C4GameController] detected the robot's move. Doing nothing.")
                    continue
                else:
                    rospy.loginfo("[C4GameController] detected the opponents's move. Making a move.")
            
            if self.game_state == "player1_win" or self.game_state == "player2_win":
                rospy.loginfo("[C4GameController] game finished")
                rospy.signal_shutdown("game over")
            
            rospy.loginfo("[C4GameController] requesting next move")
            next_move_msg = self.next_move_srv(move_made)
            # pass next_move to counter placer
            rospy.loginfo("[C4GameController] performing next move")
            place_req = PlaceCounterRequest()
            place_req.next_move.next_move = next_move_msg.next_move.next_move
            success = self.place_counter_srv(place_req)
            if success:
                rospy.loginfo("[C4GameController] successfully placed counter.")
            # check wins etc from class variables
            # if game_over:
                # signal to gui?
                # wait for decision (finished/replay)
                # if replay:
                #   reset everything somehow (a message that every resettable node subs to?)
                #   continue
                # signal a shutdown?
            
    def board_state_cb(self, msg):
        # TODO check board state for wins etc.
        pass


    def reset_state(self, msg):
        self.game_state = "playing"
    
    def move_made_check(self, msg):
        board_state = msg.board_state
        self.check_board_state(board_state)


    def check_board_state(self, board_state):
        player1 = board_state.player1
        player2 = board_state.player2
        heights = board_state.heights

        def is_win(bitboard):
            directions = [1, 7, 6, 8]
            temp_bb = 0
            for direction in directions:
                temp_bb = bitboard & (bitboard >> direction)
                if ((temp_bb & (temp_bb >> (2 * direction))) != 0):
                    return True
            return False
        
        if is_win(player1):
            rospy.loginfo("[C4GameController] red win")
            self.game_state = "player1_win"
        elif is_win(player2):
            rospy.loginfo("[C4GameController] yellow win")
            self.game_state = "player2_win"
        else:
            self.game_state = "playing"

        





if __name__ == "__main__":
    game_controller = C4GameController()
    game_controller.setup_game()
    game_controller.run()
