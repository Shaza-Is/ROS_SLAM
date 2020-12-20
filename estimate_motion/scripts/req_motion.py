
#!/usr/bin/env python

import sys
import rospy
from robot_mover.DriveToTarget.srv import *
import numpy as np

def drive_client(v, w):
    rospy.wait_for_service('robot_mover/command_robot')
    try:
         # Request velocity change to lin_x, ang_z
        req = (robot_mover.DriveToTarget.request.linear_x(v), robot_mover.DriveToTarget.request.angular_z(w))
        drive = rospy.ServiceProxy('/robot_mover/command_robot', req)
        
                
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

def unicycle(u, q, T):
    new_x = q[0] + T*u[0]*np.cos(q[2])
    new_y = q[1] + T*u[0]*np.sin(q[2])
    new_theta = q[2] + T*u[1]
    new_q = np.array([new_x, new_y, new_theta])
    return new_q

if __name__ == "__main__":
    
    simtime = 40
    T = 0.1
    t = np.arange(0, simtime, T)
    N = np.size(t)
    q_init = np.zeros(3)
    q_init[0] = 0.0
    q_init[1] = 0.0
    q_init[2] = 0.0
    u_m = np.zeros((2, N))
    q = np.zeros((3, N))
    q[:, 0] = q_init
    u_m[:, 0] = np.zeros(2)
    for i in range (1,N):

    # Compute some inputs (i.e., drive around)
        u = np.array([5, np.sin(0.1*T*i)])

    # Run the vehicle motion model
        q[:, i] = unicycle(u, q[:, i-1], T)

    # Model the proprioceptive sensors (i.e., speed and turning rate)
        u_m[0, i] = u[0] 
        u_m[1, i] = u[1] 



        print "Requesting %s+%s"%(u_m[0, i],  u_m[1, i])
        print "%s + %s = %s"%(u_m[0, i], u_m[1, i]), drive_client(u_m[0, i],  u_m[1, i]))
        rospy.spin()

    #stop after exploration
    print "Requesting %s+%s"%(0, 0)
    print "%s + %s = %s"%(0, 0, drive_client(0,  0))
    rospy.spin()
    