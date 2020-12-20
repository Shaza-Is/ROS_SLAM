#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.orientation)
    
def imu_listener():

    rospy.init_node('imu_listener')

    rospy.Subscriber("/imu", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    imu_listener()