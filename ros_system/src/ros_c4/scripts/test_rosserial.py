#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty

def servo_publisher():
    pub = rospy.Publisher('trigger_dispenser', Empty, queue_size=10)
    rospy.init_node('dispenser_triggerer', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        input("Press enter to trigger dispenser >")
        trigger_msg = Empty()
        rospy.loginfo("Triggering dispenser.....")
        pub.publish(trigger_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        servo_publisher()
    except rospy.ROSInterruptException:
        pass