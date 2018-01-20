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

#This function contains the pure pursuit algorithm, which adjusts the robot's angular
#velocity determined by a "look-ahead distance" while maintaining a constant linear speed. 
#This algorithm is used in order to move efficiently between waypoints while avoiding collisions.
def waypoint_allocation(data):
    
    #The following parameters are determined by PathOptimization.py
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

    #The following look ahead distance was chosen for a small cross track error.
    look_ahead = 0.2 #look ahead distance

    rospy.wait_for_service('/interpolate_path')
    g = rospy.ServiceProxy('/interpolate_path', InterpolatePath)

    final_distance = 1.0
   
    #Rate at which algorithm loops (10 Hz)
    rate = rospy.Rate(10)
    
    #Pure Pursuit algorithm runs until a the distance between the goal point and the robot is greater than 0.57.
    #This is to account for the robot's arm length. The center of the robot must be offset from the block in 
    #order for the KUKA arm to reach it without a collision.
    while final_distance > 0.57 and not rospy.is_shutdown():    

        closest = cp(robpos.position,trail)
        #The destination point changes at a rate determined by the look_ahead distance.
        destination = g(trail,Float32(closest.path_position.data + look_ahead)).point
        
        #Uses quaternions to interpolate the robot's rotation representation in its current fram
        posQuat = (robpos.orientation.x, robpos.orientation.y, robpos.orientation.z, robpos.orientation.w)
        QuatEuler = euler_from_quaternion(posQuat)
        angle = QuatEuler[2]
        cos = math.cos(angle)
        sin = math.sin(angle)

        #Constantly updates the differences between the destination's coordinates and the robot's coordinates
        deltax = destination.x - robpos.position.x
        deltay = destination.y - robpos.position.y
        deltaz = destination.z
        
        #This point has coordinates based on the robot's current frame
        #For example, if the robot is tilted 30 degrees to the left, the destination's x and y
        #values would change relative to the robot if the robot is treated as the origin (0,0)
        transformed_point = Point(deltax * cos + deltay * sin, deltax * -sin + deltay * cos, deltaz)

        xt = transformed_point.x
        yt = transformed_point.y

        #Determines the angular velocity of the robot in order for it to reach the destination in a curved motion
        #which avoids collisions with ungathered blocks while reaching the goal point without too much of an arc.
        while eliminating cross track error.
        if (((xt * xt) + (yt * yt)) != 0.0):
            heading = (-2.0 * yt) / ((xt * xt) + (yt * yt))
        else:
            heading = 0.1

        diffx = robpos.position.x - final_point.x
        diffy = robpos.position.y - final_point.y
        diffz = robpos.position.z - final_point.z
        
        #Computes the euclidean distance. This is used in the while loop condition.
        final_distance = math.sqrt((diffx * diffx) + (diffy * diffy) + (diffz * diffz))
        print final_distance

        msg = Twist() 

        linear = 1.0
                
        msg.linear.x = 0
        #Robot moves in the y axis of its current frame at a constant speed
        msg.linear.y = linear
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        #heading determined by pure pursuit algorithm
        msg.angular.z = heading * linear * 0.4

        follow.publish(msg)
        rate.sleep()
        
    #Stops the robot by setting its angular and linear velocities to zero
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
