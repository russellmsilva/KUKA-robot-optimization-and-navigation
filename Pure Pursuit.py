#!/usr/bin/env python
import rospy
import math
import numpy as np
import sys
from tf.transformations import euler_from_quaternion
from cvxopt import matrix, solvers
from foundations_hw3.msg import *
from foundations_hw3.srv import *  
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

n = 0
trail = []
robpos = Point(0.0, 0.0, 0.0)
final_point = Point(0.0, 0.0, 0.0)

def waypoint_allocation(data):

    global trail
    global final_point
    trail = data.path
    final_point = trail[-1]

    #rospy.wait_for_service('/vrep/bucket/pos/get')
    #bucket = rospy.ServiceProxy('/vrep/bucket/pos/get', GetPosition) 

    #start = rospy.wait_for_message('/vrep/youbot/base/pose', Pose) 

    #cost function

    follow = rospy.Publisher('/vrep/youbot/base/cmd_vel', Twist, queue_size = 10)
 
    closest = Point()
    destination = Point()

    rospy.wait_for_service('/closest_point_path')
    cp = rospy.ServiceProxy('/closest_point_path', ClosestPointPath)

    look_ahead = 0.2 #look ahead distance

    rospy.wait_for_service('/interpolate_path')
    g = rospy.ServiceProxy('/interpolate_path', InterpolatePath)

    final_distance = 1.0
   
    rate = rospy.Rate(10)
    while final_distance > 0.57 and not rospy.is_shutdown():    

        closest = cp(robpos.position,trail)
        destination = g(trail,Float32(closest.path_position.data + look_ahead)).point

        posQuat = (robpos.orientation.x, robpos.orientation.y, robpos.orientation.z, robpos.orientation.w)
        QuatEuler = euler_from_quaternion(posQuat)
        angle = QuatEuler[2]
        cos = math.cos(angle)
        sin = math.sin(angle)

        deltax = destination.x - robpos.position.x
        deltay = destination.y - robpos.position.y
        deltaz = destination.z

        transformed_point = Point(deltax * cos + deltay * sin, deltax * -sin + deltay * cos, deltaz)

        xt = transformed_point.x
        yt = transformed_point.y

        if (((xt * xt) + (yt * yt)) != 0.0):
            heading = (-2.0 * yt) / ((xt * xt) + (yt * yt))
        else:
            heading = 0.1

        diffx = robpos.position.x - final_point.x
        diffy = robpos.position.y - final_point.y
        diffz = robpos.position.z - final_point.z
        final_distance = math.sqrt((diffx * diffx) + (diffy * diffy) + (diffz * diffz))
        print final_distance

        msg = Twist() 

        linear = 1.0
                
        msg.linear.x = 0
        msg.linear.y = linear
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = heading * linear * 0.4

        follow.publish(msg)
        rate.sleep()

    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0

    follow.publish(msg)

    return FollowPathResponse()
    
def callback(data):

    global robpos
    global final_point
    robpos = data
    
def main():
    
    rospy.init_node('p2b')
    rospy.Service('p2b', FollowPath, waypoint_allocation)
    rospy.Subscriber('/vrep/youbot/base/pose', Pose, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
