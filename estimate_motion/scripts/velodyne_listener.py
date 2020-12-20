#!/usr/bin/env python
import rospy
import numpy as np
import pcl_msgs
from sensor_msgs.msg import PointCloud2 as pc2
from std_msgs.msg import String
#import sensor_msgs.point_cloud2 as pc2
import pcl

def callback(data):
    rospy.loginfo("I heard %s", data)
    pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    
def velodyne_listener():

    rospy.init_node('velodyne_listener')

    rospy.Subscriber("/velodyne_points", pc2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    velodyne_listener()