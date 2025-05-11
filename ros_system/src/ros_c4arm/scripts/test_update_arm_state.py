import rospy
from open_manipulator_msgs.msg import KinematicsPose, JointPosition, OpenManipulatorState

from sensor_msgs.msg import JointState

from ros_c4msgs.msg import ArmState

def state_cb(msg):
    armState = ArmState()
    if msg.open_manipulator_moving_state == msg.IS_MOVING:
        armState.status = armState.ACTIVE
    else:
        armState.status = armState.IDLE
    pub.publish(armState)


if __name__ == "__main__":
    rospy.init_node("test_update_arm_state", anonymous=True)
    rospy.Subscriber(name="/states", data_class=OpenManipulatorState, callback=state_cb)
    pub = rospy.Publisher("c4/arm_state", ArmState, queue_size=10)
    rospy.spin()
