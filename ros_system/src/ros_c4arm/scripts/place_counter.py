#!/usr/bin/env python3

import geometry_msgs.msg
import rospy
import geometry_msgs
import moveit_commander
import sys
from tf.transformations import quaternion_from_euler
from math import radians

from std_msgs.msg import Empty
from ros_c4msgs.srv import GetCounterLocation, PlaceCounterRequest, PlaceCounterResponse, PlaceCounter
from ros_c4msgs.msg import NextMove

class CounterPlacer():
    def __init__(self):
        rospy.init_node("counter_placer", anonymous=True)

        # MoveIt setup
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_arm.set_planning_time(10)
        self.move_group_arm.set_num_planning_attempts(10)
        self.move_group_arm.set_max_velocity_scaling_factor(0.4)
        self.move_group_arm.set_max_acceleration_scaling_factor(0.4)
        self.move_group_arm.set_planner_id("TRRT")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        
        # Subscribers and publishers
        self.dispenser_pub = rospy.Publisher("trigger_dispenser", Empty, queue_size=1)
        self.provided_service = rospy.Service(
            name="/c4/place_counter",
            service_class=PlaceCounter,
            handler=self.place_counter
        )

        # Class variables
        self.colour_searching = "red"
        
        self.column = None
        self.column_y_positions = [
                 0.092, # column 0
                 0.062, # column 1
                 0.032, # column 2
                 0.000, # column 3
                -0.032, # column 4
                -0.062, # column 5
                -0.092  # column 6
        ]

        # Initially move to the observation pose
        self.add_box()
        self.observation_pose()
        rospy.sleep(2)
        while not rospy.is_shutdown():
            rospy.spin()
    
    def place_counter(self, request):
        self.colour_searching = rospy.get_param("/c4/robot_colour", "red")
        
        self.column = request.next_move.next_move

        rospy.loginfo("going to observation pose.....")
        trigger_msg = Empty()
        rospy.loginfo("Triggering dispenser.....")
        self.dispenser_pub.publish(trigger_msg)
        self.open_gripper()
        self.observation_pose()
        rospy.sleep(3)

        rospy.loginfo("going to counter.....")
        self.pre_pick_position()

        self.close_gripper()
        rospy.sleep(1)
        rospy.loginfo("going to drop pose.....")
        self.safe_position_drop()
        self.pre_drop_pose()
        self.scene.remove_world_object("c4board")
        self.drop_pose()
        self.open_gripper()
        self.pre_drop_pose()
        self.add_box()
        self.observation_pose()

        return True


    def pre_pick_position(self):
        rospy.wait_for_service("/c4/get_counter_location")
        get_counter_location = rospy.ServiceProxy("/c4/get_counter_location", GetCounterLocation)
        response_point = get_counter_location(self.colour_searching)
        point = response_point.point

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

        self.move_group_arm.set_start_state_to_current_state()
        self.move_group_arm.set_pose_target(pose_goal)

        self.move_group_arm.go(wait=True)
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()
        self.current_point = (point.x, point.y, point.z)
        self.go_to_counter_pickup(point=point)

    def go_to_counter_pickup(self, point):
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.position.x = point.x + 0.005
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

        pose_goal.pose.orientation.x = rotation_q[0]
        pose_goal.pose.orientation.y = rotation_q[1]
        pose_goal.pose.orientation.z = rotation_q[2]
        pose_goal.pose.orientation.w = rotation_q[3]

        self.move_group_arm.set_start_state_to_current_state()
        self.move_group_arm.set_pose_target(pose_goal)

        success = self.move_group_arm.go(wait=True)
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

        pose_goal.pose.position.x = 0.23
        pose_goal.pose.position.y = self.column_y_positions[self.column]
        pose_goal.pose.position.z = 0.23 # 227

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
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

    def pre_drop_pose(self):
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"

        pose_goal.pose.position.x = 0.225
        pose_goal.pose.position.y = self.column_y_positions[self.column]
        pose_goal.pose.position.z = 0.26

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
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

    def safe_position_drop(self):
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.position.x = 0.15
        pose_goal.pose.position.y = self.column_y_positions[self.column]
        pose_goal.pose.position.z = 0.2

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

if __name__ == "__main__":
    CounterPlacer()
