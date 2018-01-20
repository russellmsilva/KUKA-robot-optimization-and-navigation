#!/usr/bin/env python
import rospy
import math
import numpy
import sys
from foundations_hw2.msg import * 
from std_msgs.msg import Float64 
from hw2.srv import *
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Point

'''
The following callback function performs a forward kinematics calculation
relating to a jacobian matrix which contains information on the robot arm's joint angles
and link lengths. This KUKA robot arm has four links and five joint angles. The grasper
of the KUKA robot arm is treated as an end effector which represents the end result
of this calculation (an x,y,z coordinate).
'''
def callback(req):
    rospy.loginfo(req)
    
    #theta1-theta5 represent the joint angles
    theta1 = req.joint_service.angles[0]
    theta2 = req.joint_service.angles[1]
    theta3 = req.joint_service.angles[2]
    theta4 = req.joint_service.angles[3]
    theta5 = req.joint_service.angles[4]
   
    t1 = theta1 + math.pi / 2.0
    t2 = theta2 + math.pi / 2.0
    
    c1 = math.cos(t1)
    c2 = math.cos(t2 + theta3)
    c3 = math.cos(t2)
    c4 = math.cos(t2 + theta3 + theta4)

    s1 = math.sin(t1)
    s2 = math.sin(t2 + theta3)
    s3 = math.sin(t2)
    s4 = math.sin(t2 + theta3 + theta4)
    
    #a1-a4 represent the link lengths
    a1 = 0.033
    a2 = 0.155
    a3 = 0.135
    a4 = 0.2175

    joint_service = Point()
    #x-coordinate of the end effector
    joint_service.x = (c1 * ((435 * c4) + (270 * c2) + (310 * c3) + 66))/2000.0
    #y-coordinate of the end effector
    joint_service.y = (s1 * ((435 * c4) + (270 * c2) + (310 * c3) + 66))/2000.0
    #z-coordinate of the end effector
    joint_service.z = (87 * s4) / 400.0 + (27 * s2) / 200.0 + (31 * s3) / 200.0 + 0.101

    return p1bResponse(joint_service) 

def main():
    
    rospy.init_node('p1b')
    
    service = rospy.Service ('p1b', p1b, callback)
    
    #pub.publish(message)

    rospy.spin()

if __name__ == '__main__':
    main()
