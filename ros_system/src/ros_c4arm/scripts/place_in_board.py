#!/usr/bin/env python3

import geometry_msgs.msg
import moveit_msgs.msg
import rospy
import geometry_msgs
import moveit_commander
import moveit_msgs
import sys
from tf.transformations import quaternion_from_euler
from math import radians
import quaternion
from shape_msgs.msg import SolidPrimitive
from octomap_msgs.msg import Octomap

from std_srvs.srv import Empty

from ros_c4msgs.srv import GetCounterLocation

class CounterPicker():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("counter_picker", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        self.move_group_arm = moveit_commander.MoveGroupCommander(group_name)
        self.move_group_arm.set_planning_time(10)
        self.move_group_arm.set_num_planning_attempts(10)

        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        # self.move_group_arm.set_planner_id("BFMT")

        self.clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
        self.octomap_pub = rospy.Publisher("/octomap_binary", Octomap, queue_size=1, latch=True)
        self.saved_octomap = None

        self.add_box()
        # print("press enter to proceed")
        # input()

        # while not rospy.is_shutdown():
        rospy.loginfo("going to observation pose.....")
        self.open_gripper()
        self.observation_pose()
        rospy.sleep(2)

        rospy.loginfo("going to counter.....")
        self.pre_pick_position()

        self.close_gripper()
        rospy.sleep(1)
        # rospy.loginfo("going to observation pose.....")
        self.pre_drop_pose()
        self.scene.remove_world_object("c4board")
        rospy.sleep(0.5)
        self.drop_pose()
        self.open_gripper()
        rospy.sleep(0.5)
        self.pre_drop_pose()
        rospy.sleep(0.5)
        self.add_box()
        self.observation_pose()

    def pre_pick_position(self):
        rospy.wait_for_service("/c4/get_counter_location")
        get_counter_location = rospy.ServiceProxy("/c4/get_counter_location", GetCounterLocation)
        response_point = get_counter_location()
        point = response_point.point
        # point = rospy.wait_for_message("testing/counter_location", geometry_msgs.msg.Point, timeout=30.0)

        # go to a pose right over the desired location
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.position.x = point.x# + 0.005
        fudge = 1 if point.y < 0.0 else 0
        pose_goal.pose.position.y = point.y - 0.015 * fudge
        pose_goal.pose.position.z = 0.0
        # print(pose_goal.pose.position)

        rotation = [0, 85, 0]
        rotation = [radians(rot) for rot in rotation]

        rotation_q = quaternion_from_euler(rotation[0],rotation[1],rotation[2])
        pose_goal.pose.orientation.x = rotation_q[0]
        pose_goal.pose.orientation.y = rotation_q[1]
        pose_goal.pose.orientation.z = rotation_q[2]
        pose_goal.pose.orientation.w = rotation_q[3]

        # self.clear_octomap()
        # rospy.sleep(10)
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
        rospy.wait_for_service("/clear_octomap")
        


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
        pose_goal.pose.position.z = point.z + 0.028# + scale_factor
        # print(pose_goal.pose.position)

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
        rospy.logwarn("Clearing octomap.....")
        # self.save_octomap()
        # self.clear_octomap()
        # rospy.sleep(10)
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
        
        # self.restore_octomap()

    def observation_pose(self):
        """Go to  a sort of observation pose."""
        joint_goal = self.move_group_arm.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = -0.877
        joint_goal[2] = 0.265
        joint_goal[3] = 1.914
        # joint_goal[5] = 0.0
        # joint_goal[6] = 0.0
        self.move_group_arm.go(joint_goal, wait=True)
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

    def drop_pose(self):
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.position.x = 0.232
        pose_goal.pose.position.y = 0.092
        pose_goal.pose.position.z = 0.227

        rotation = [0, 0, 0]
        rotation = [radians(rot) for rot in rotation]

        rotation_q = quaternion_from_euler(rotation[0],rotation[1],rotation[2])

        pose_goal.pose.orientation.x = rotation_q[0]
        pose_goal.pose.orientation.y = rotation_q[1]
        pose_goal.pose.orientation.z = rotation_q[2]
        pose_goal.pose.orientation.w = rotation_q[3]
        self.move_group_arm.set_start_state_to_current_state()
        self.move_group_arm.set_pose_target(pose_goal)
        success = self.move_group_arm.go(wait=True)
        if success:
            print("Planning successful.")
        else:
            print("Planning failed.")
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

    def pre_drop_pose(self):
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.position.x = 0.225
        pose_goal.pose.position.y = 0.092
        pose_goal.pose.position.z = 0.25

        rotation = [0, 0, 0]
        rotation = [radians(rot) for rot in rotation]

        rotation_q = quaternion_from_euler(rotation[0],rotation[1],rotation[2])

        pose_goal.pose.orientation.x = rotation_q[0]
        pose_goal.pose.orientation.y = rotation_q[1]
        pose_goal.pose.orientation.z = rotation_q[2]
        pose_goal.pose.orientation.w = rotation_q[3]
        self.move_group_arm.set_start_state_to_current_state()
        self.move_group_arm.set_pose_target(pose_goal)
        success = self.move_group_arm.go(wait=True)
        if success:
            print("Planning successful.")
        else:
            print("Planning failed.")
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

    def add_box(self):
        # rospy.sleep(1)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.21
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.1
        box_name = "c4board"

        self.scene.add_box(
            box_name,
            box_pose,
            size=(0.015, 0.3, 0.2)
        )
        # box = moveit_msgs.msg.CollisionObject()
        # box.header.frame_id = "world"
        # box.id = "c4board"

        # box_primitive = SolidPrimitive()
        # box_primitive.type = SolidPrimitive.BOX
        # box_primitive.dimensions = [0.15, 0.26, 0.28]

        # box_pose_msg = geometry_msgs.msg.Pose()
        # box_pose_msg.orientation.w = 1.0
        # box_pose_msg.position.x = 0.25
        # box_pose_msg.position.y = 0.0
        # box_pose_msg.position.z = -0.1

        # box.primitives.append(box_primitive)
        # box.primitive_poses.append(box_pose_msg)
        # box.operation = box.ADD

        # self.scene.apply_collision_objects(box)
        # rospy.logwarn("added box")

    def disable_collisions(self):
        pass

    def enable_collisions(self):
        pass

    def save_octomap(self):
        octomap = rospy.wait_for_message("/octomap_binary", Octomap)
        self.saved_octomap = octomap
        rospy.loginfo("[CounterPicker] saved Octomap.")

    def restore_octomap(self):
        if self.saved_octomap is None:
            rospy.logerr("[CounterPicker] no saved Octomap to restore!!!!")
        self.saved_octomap.header.stamp = rospy.Time.now()
        self.octomap_pub.publish(self.saved_octomap)
        rospy.loginfo("[CounterPicker] restored Octomap.")

if __name__ == "__main__":
    CounterPicker()
