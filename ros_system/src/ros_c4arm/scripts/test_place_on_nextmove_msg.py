#!/usr/bin/env python3

import rospy
from ros_c4msgs.msg import MoveMade, NextMove
from random import randint
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from open_manipulator_msgs.msg import OpenManipulatorState

from std_msgs.msg import Bool

import threading

OPEN_GRIPPER = 0.01
CLOSE_GRIPPER = -0.006

pickup_done = False

def go_to_column(column):
    column_location = column if column else 3
    rospy.wait_for_service("goal_task_space_path_position_only")
    try:
        goal_task_space_path_position_only = rospy.ServiceProxy("goal_task_space_path_position_only", SetKinematicsPose)
        srv = SetKinematicsPoseRequest()
        srv.end_effector_name = "gripper"

        srv.kinematics_pose.pose.position.x = 0.238
        # srv.kinematics_pose.pose.position.y = 0
        srv.kinematics_pose.pose.position.z = 0.23

        if column == 0:
            srv.kinematics_pose.pose.position.y = 0.096
        elif column == 1:
            srv.kinematics_pose.pose.position.y = 0.065
        elif column == 2:
            srv.kinematics_pose.pose.position.y = 0.034
        elif column == 3:
            srv.kinematics_pose.pose.position.y = 0
        elif column == 4:
            srv.kinematics_pose.pose.position.y = -0.034
        elif column == 5:
            srv.kinematics_pose.pose.position.y = -0.065
        elif column == 6:
            srv.kinematics_pose.pose.position.y = -0.096

        srv.path_time = 2

        response = goal_task_space_path_position_only.call(srv)
        print(response)

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def pickup_pose():
    rospy.wait_for_service('/goal_joint_space_path')
    try:
        goal_joint_space_path_service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        goal_joint_space_path_request_object = SetJointPositionRequest()

        goal_joint_space_path_request_object.planning_group = 'arm'
        goal_joint_space_path_request_object.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        goal_joint_space_path_request_object.joint_position.position = [1.155, 0.816, 0.048, 0.652]
        goal_joint_space_path_request_object.joint_position.max_accelerations_scaling_factor = 1.0
        goal_joint_space_path_request_object.joint_position.max_velocity_scaling_factor = 1.0
        goal_joint_space_path_request_object.path_time = 2.5
        result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)
        print(result)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def home_pose():
    rospy.wait_for_service('/goal_joint_space_path')
    try:
        goal_joint_space_path_service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        goal_joint_space_path_request_object = SetJointPositionRequest()

        goal_joint_space_path_request_object.planning_group = 'arm'
        goal_joint_space_path_request_object.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        goal_joint_space_path_request_object.joint_position.position = [0.0, -1, 0.3, 0.7]
        goal_joint_space_path_request_object.joint_position.max_accelerations_scaling_factor = 1.0
        goal_joint_space_path_request_object.joint_position.max_velocity_scaling_factor = 1.0
        goal_joint_space_path_request_object.path_time = 1.5
        result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)
        print(result)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def set_gripper(value):
    goal_tool_control = rospy.ServiceProxy("goal_tool_control", SetJointPosition)
    srv = SetJointPositionRequest()
    srv.planning_group = "gripper"
    srv.joint_position.joint_name = ["gripper"]
    srv.joint_position.position = [value]
    srv.joint_position.max_accelerations_scaling_factor = 1
    srv.joint_position.max_velocity_scaling_factor = 1
    srv.path_time = 2

    response = goal_tool_control(srv)
    print(response)

def move_made_cb(msg):
    global pickup_started, pickup_finished
    if msg.move_made == msg.PLAYER1_MOVE:
        rospy.logerr("Player 2 made move.")
        return
    if not pickup_started and pickup_finished:
        pickup_started = True
        pickup_finished = False
        set_gripper(OPEN_GRIPPER)
        rospy.sleep(3)
        pickup_pose()
        rospy.sleep(3)
        set_gripper(CLOSE_GRIPPER)
        rospy.sleep(3)
        home_pose()
        rospy.sleep(3)
        pickup_started = False
        pickup_finished = True
    else:
        rospy.logerr("Already picking up a counter.")

class place_node:
    def __init__(self):
        self.is_busy = False
        self.lock = threading.Lock()
        self.pickup_done = False
        self.next_move = None
        self.new_move = False
        rospy.init_node("test_place_on_nextmove_msg", anonymous=True)
        rospy.Subscriber(name="c4/next_move", data_class=NextMove, callback=self.callback)
        rospy.Subscriber(name="c4/pickup_state", data_class=Bool, callback=self.state_callback)

    def state_callback(self, msg):
        self.pickup_done = msg.data
        rospy.logerr(f"CALLBACK FOR PICKUP STATE {self.pickup_done}")
        if self.pickup_done and self.new_move:
            self.place(self.next_move)

    def callback(self, msg):
        self.next_move = msg.next_move
        self.new_move = True
        if self.pickup_done:
            self.place(self.next_move)
        # with self.lock:
        #     if not self.pickup_done:
        #         rospy.logwarn("Not finished picking up counter.")
        #         return
        #     if self.is_busy:
        #         rospy.logwarn("Already placing up a counter.")
        #         return
            
        #     self.is_busy = True

        # thread = threading.Thread(target=self.place, args=(msg.next_move,))
        # thread.start()

    def place(self, next_move):
        rospy.loginfo("Starting counter placing.")
        go_to_column(next_move)
        rospy.sleep(3)
        set_gripper(OPEN_GRIPPER)
        rospy.sleep(2)
        home_pose()
        rospy.sleep(3)

        with self.lock:
            self.is_busy = False
        rospy.loginfo("Finished counter placing.")
        self.pickup_done = False
        self.next_move = None
        self.new_move = False


if __name__ == "__main__":
    node = place_node()
    rospy.spin()
