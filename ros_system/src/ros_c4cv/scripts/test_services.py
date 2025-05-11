#!/usr/bin/env python3
import rospy
# import argparse
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from open_manipulator_msgs.msg import OpenManipulatorState

# parser = argparse.ArgumentParser()
# parser.add_argument("-c", "--column", required=False)
# args = parser.parse_args()

OPEN_GRIPPER = 0.01
CLOSE_GRIPPER = -0.006

is_moving = False

def go_to_column(column):
    column_location = column if column else 3
    rospy.wait_for_service("goal_task_space_path_position_only")
    try:
        goal_task_space_path_position_only = rospy.ServiceProxy("goal_task_space_path_position_only", SetKinematicsPose)
        srv = SetKinematicsPoseRequest()
        srv.end_effector_name = "gripper"

        srv.kinematics_pose.pose.position.x = 0.238
        # srv.kinematics_pose.pose.position.y = 0
        srv.kinematics_pose.pose.position.z = 0.225

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
        goal_joint_space_path_request_object.path_time = 2.0
        result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)
        print(result)
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
        goal_joint_space_path_request_object.path_time = 3.0
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

def state_cb(msg):
    global is_moving
    open_manipulator_moving_state = msg.open_manipulator_moving_state
    if open_manipulator_moving_state == msg.IS_MOVING:
        is_moving = True
    else:
        is_moving = False

if __name__ == "__main__":
    rospy.init_node("test_services")
    rospy.wait_for_service("goal_task_space_path_position_only")
    rospy.wait_for_service("goal_joint_space_path")
    rospy.wait_for_service("goal_tool_control")

    rospy.Subscriber(name="/states", data_class=OpenManipulatorState, callback=state_cb)

    home_pose()
    rospy.sleep(3)
    set_gripper(OPEN_GRIPPER)
    keep_entering = True
    while(keep_entering):
        choice = -1
        while int(choice) not in range(0, 7):
            choice = input("Enter a column 0->6: ")
            if choice == "q":
                keep_entering = False
                break
        if keep_entering:
            print("going to pickup pose")
            pickup_pose()
            rospy.sleep(3)
            set_gripper(CLOSE_GRIPPER)
            rospy.sleep(3)
            print(f"Moving to column {choice}")
            home_pose()
            rospy.sleep(3)
            go_to_column(int(choice))
            rospy.sleep(5)
            set_gripper(OPEN_GRIPPER)
            rospy.sleep(3)
            home_pose()
            rospy.sleep(3)
