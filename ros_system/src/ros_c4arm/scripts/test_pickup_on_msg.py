#!/usr/bin/env python3

import rospy
from ros_c4msgs.msg import MoveMade, NextMove
from random import randint
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from open_manipulator_msgs.msg import OpenManipulatorState

from std_msgs.msg import Bool, String

import threading

pickup_started = False
pickup_finished = True
OPEN_GRIPPER = 0.01
CLOSE_GRIPPER = -0.006

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

class pickup_node:
    def __init__(self):
        self.is_busy = False
        self.lock = threading.Lock()
        rospy.init_node("test_pickup_on_msg", anonymous=True)
        rospy.Subscriber(name="c4/move_made", data_class=MoveMade, callback=self.callback)
        self.done_pub = rospy.Publisher("c4/pickup_state", Bool, queue_size=1)

    def callback(self, msg):
        with self.lock:
            if msg.move_made == msg.PLAYER1_MOVE:
                rospy.logwarn("Player 2 made a move. No need to do anything.")
                return

            if self.is_busy:
                rospy.logwarn("Already picking up a counter.")
                return
            
            self.is_busy = True

        thread = threading.Thread(target=self.pickup)
        thread.start()

    def pickup(self):
        rospy.loginfo("Starting counter pickup.")
        set_gripper(OPEN_GRIPPER)
        # rospy.sleep(3)
        pickup_pose()
        rospy.sleep(3)
        set_gripper(CLOSE_GRIPPER)
        rospy.sleep(3)
        home_pose()
        rospy.sleep(3)

        with self.lock:
            self.is_busy = False
        rospy.loginfo("Finished counter pickup.")
        message = Bool()
        message.data = True
        self.done_pub.publish(message)


if __name__ == "__main__":
    node = pickup_node()
    rospy.spin()
