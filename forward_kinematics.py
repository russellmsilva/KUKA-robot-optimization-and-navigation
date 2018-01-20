#!/usr/bin/env python
import rospy
import math
import numpy
import sys
from foundations_hw2.msg import * 
from std_msgs.msg import Float64 
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from hw2.srv import *
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Point

def callback(req):
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

    jac = Float64MultiArray()

    im = MultiArrayLayout()
    im.dim = [MultiArrayDimension('rows', 5, 15), MultiArrayDimension('columns', 3, 3)]
   
    c1r1 = -(math.sin(s1)*(435*math.cos(c4) + 270*math.cos(c2) + 310*math.cos(c3) + 66))/2000
    c1r2 = (math.cos(c1)*(435*math.cos(c4) + 270*math.cos(c2) + 310*math.cos(c3) + 66))/2000
    c1r3 = 0

    c2r1 = -(math.cos(c1)*(435*math.sin(s4) + 270*math.sin(s2) + 310*math.sin(s3)))/2000
    c2r2 = -(math.sin(s1)*(435*math.sin(s4) + 270*math.sin(s2) + 310*math.sin(s3)))/2000
    c2r3 = (87*math.cos(c4))/400 + (27*math.cos(c2))/200 + (31*math.cos(c3))/200

    c3r1 = -(math.cos(c1)*(435*math.sin(s4) + 270*math.sin(s2)))/2000
    c3r2 = -(math.sin(s1)*(435*math.sin(s4) + 270*math.sin(s2)))/2000
    c3r3 = (87*math.cos(c4))/400 + (27*math.cos(c2))/200

    c4r1 = -(87*math.sin(s4)*math.cos(c1))/400
    c4r2 = -(87*math.sin(s4)*math.sin(s1))/400
    c4r3 = (87*math.cos(c4))/400
 
    c5r1 = 0
    c5r2 = 0
    c5r3 = 0

    inputted = [c1r1, c1r2, c1r3, c2r1, c2r2, c2r3, c3r1, c3r2, c3r3, c4r1, c4r2, c4r3, c5r1, c5r2, c5r3]
    jac.layout = im
    jac.data = inputted

    return p2bResponse(jac) 

def main():
    
    rospy.init_node('p2b')
    
    service = rospy.Service ('p2b', p2b, callback)
    
    #pub.publish(message)

    rospy.spin()

if __name__ == '__main__':
    main()
