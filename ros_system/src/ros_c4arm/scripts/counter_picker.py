#!/usr/bin/env python3

import geometry_msgs.msg
import rospy
import geometry_msgs
import moveit_commander
import moveit_msgs
import sys
from tf.transformations import quaternion_from_euler
from math import radians
import quaternion

class CounterPicker():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("counter_picker", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        self.move_group_arm = moveit_commander.MoveGroupCommander(group_name)
        self.move_group_arm.set_planning_time(10)
        self.move_group_arm.set_num_planning_attempts(10)

        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        # self.move_group_arm.set_planner_id("BFMT")

        while not rospy.is_shutdown():
            rospy.loginfo("going to observation pose.....")
            self.open_gripper()
            self.observation_pose()
            rospy.sleep(2)

            rospy.loginfo("going to counter.....")
            self.pre_pick_position()

            self.close_gripper()
            rospy.sleep(1)
            rospy.loginfo("going to observation pose.....")
            self.observation_pose()
            self.open_gripper()

    def pre_pick_position(self):
        point = rospy.wait_for_message("testing/counter_location", geometry_msgs.msg.Point, timeout=30.0)

        # go to a pose right over the desired location
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.position.x = point.x# + 0.005
        fudge = 1 if point.y < 0.0 else 0
        pose_goal.pose.position.y = point.y - 0.015 * fudge
        pose_goal.pose.position.z = 0.0
        print(pose_goal.pose.position)

        rotation = [0, 85, 0]
        rotation = [radians(rot) for rot in rotation]

        rotation_q = quaternion_from_euler(rotation[0],rotation[1],rotation[2])
        pose_goal.pose.orientation.x = rotation_q[0]
        pose_goal.pose.orientation.y = rotation_q[1]
        pose_goal.pose.orientation.z = rotation_q[2]
        pose_goal.pose.orientation.w = rotation_q[3]

        self.move_group_arm.set_start_state_to_current_state()
        self.move_group_arm.set_pose_target(pose_goal)

        self.move_group_arm.go(wait=True)
        # if success:
        #     print("Planning successful.")
        # else:
        #     print("Planning failed.")
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()
        self.go_to_counter_pickup(point=point)

    def go_to_counter_pickup(self, point):
        # point = rospy.wait_for_message("testing/counter_location", geometry_msgs.msg.Point, timeout=30.0)

        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.position.x = point.x# + 0.005
        fudge = 1 if point.y < 0.0 else 0
        pose_goal.pose.position.y = point.y - 0.015 * fudge
        if point.x >= 0.2:
            scale_factor = 0.038
        elif 0.15 <= point.x < 0.2:
            scale_factor = 0.033
        else:
            scale_factor = 0.03
        pose_goal.pose.position.z = point.z + 0.03# + scale_factor
        print(pose_goal.pose.position)

        rotation = [0, 85, 0]
        rotation = [radians(rot) for rot in rotation]

        rotation_q = quaternion_from_euler(rotation[0],rotation[1],rotation[2])
        # rotation_q = quaternion(rotation_q)
        # rotation_q.norm()
        # print(rotation_q)

        pose_goal.pose.orientation.x = rotation_q[0]
        pose_goal.pose.orientation.y = rotation_q[1]
        pose_goal.pose.orientation.z = rotation_q[2]
        pose_goal.pose.orientation.w = rotation_q[3]

        self.move_group_arm.set_start_state_to_current_state()
        # self.move_group_arm.set_goal_tolerance(0.005)
        self.move_group_arm.set_pose_target(pose_goal)

        # plan = self.move_group_arm.plan()

        # if plan:
        #     joint_angles = plan.trajectory.joint_trajectory.points[-1].positions
        #     print(joint_angles)
        #     return

        # rospy.logwarn(f"going to: [{pose_goal.pose.position.x}, {pose_goal.pose.position.y}, {pose_goal.pose.position.z}]")
        # rospy.logwarn("Press enter to move.")
        # input()

        success = self.move_group_arm.go(wait=True)
        if success:
            print("Planning successful.")
        else:
            print("Planning failed.")
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

    def observation_pose(self):
        """Go to  a sort of observation pose."""
        joint_goal = self.move_group_arm.get_current_joint_values()
        joint_goal[0] = 0.077
        joint_goal[1] = -0.877
        joint_goal[2] = 0.265
        joint_goal[3] = 1.914
        # joint_goal[5] = 0.0
        # joint_goal[6] = 0.0
        self.move_group_arm.go(joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

    def close_gripper(self):
        self.move_group_gripper.set_named_target("close_counter")
        self.move_group_gripper.go(wait=True)
        self.move_group_gripper.stop()
        self.move_group_gripper.clear_pose_targets()

    def open_gripper(self):
        self.move_group_gripper.set_named_target("open")
        self.move_group_gripper.go(wait=True)
        self.move_group_gripper.stop()
        self.move_group_gripper.clear_pose_targets()

if __name__ == "__main__":
    CounterPicker()
