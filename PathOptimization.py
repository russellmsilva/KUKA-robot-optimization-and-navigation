#!/usr/bin/env python
import rospy
import math
import numpy as np
import sys
from cvxopt import matrix, solvers
from foundations_hw3.msg import *
from foundations_hw3.srv import *  
from std_msgs.msg import Float64 
from geometry_msgs.msg import Twist 
from geometry_msgs.m
sg import Point
from geometry_msgs.msg import Pose

'''
The following callback function computes the entirety of the path planning.
The callback function takes in the number of blocks, as well as a first and last block
as parameters. Because of this, a more efficient path can be planned by computing the
minimum distances for each combination of the five blocks. This was not done in order 
to conserve computational power.
'''
def callback(data):
    #data.num_waypoints represents the number of blocks in the work space
    n = data.num_waypoints
    #obtains the position of the first block
    start_pos= data.start
    #obtains the position of the last block
    goal_pos = data.goal

    rospy.wait_for_service('compute_cost')
    cost = rospy.ServiceProxy('compute_cost', ComputeCost) 

    points = [start_pos]

    #Sets waypoints for the path, and adds each x,y point to an array
    for num in range (1, n-1):
        difference_x = (goal_pos.x - start_pos.x) / ((n-1) * 1.0)
        difference_y = (goal_pos.y - start_pos.y) / ((n-1) * 1.0)
        points.append(Point(start_pos.x + difference_x * (num * 1.0), start_pos.y + difference_y * (num * 1.0), 0.0))

    points.append(goal_pos)

    for a in range (0, n):
        print(points[a])

    #Constants used a step sizes in the optimization algorithm
    diff = 0.0001
    gm = 0.01

    latest_cost = 2.0
    starting_cost = 0.0
    
    '''
    Optimization algorithm. The operations within this algorithm loop
    until a difference between two computed consecutive path costs reaches below 0.1
    This represents a "plateau" in the data in which the maximum efficiency
    of the path has neared a maximum.
    '''
    while (abs(latest_cost - starting_cost) > 0.1):
        points_temp_x = points[:]
        points_temp_y = points[:]

        delta_x = []
        delta_y = []

        print latest_cost
        print starting_cost
        for i in range (1, n-1):
            x_temp = points[i].x + diff
            y_temp = points[i].y + diff
            points_temp_x[i] = Point(x_temp, points[i].y, 0.0)
            points_temp_y[i] = Point(points[i].x, y_temp, 0.0)
            delta_x.append(x_temp - cost(points_temp_x).cost)      
            delta_y.append(y_temp - cost(points_temp_y).cost)    

        for j in range (1, n-1):
            points[j].x = points[j].x - gm * ((delta_x[j-1]) / diff)
            points[j].y = points[j].y - gm * ((delta_y[j-1]) / diff)

        starting_cost = latest_cost

        latest_cost = cost(points).cost

    for b in range (0, n):
       print(points[b])

    path = points

    return FindPathResponse(path)


def main():
    #Initializes the ROS node as an individual program.
    rospy.init_node('p2a')
    #Establishes the program as a service that can be used by other ROS nodes.
    rospy.Service('p2a', FindPath, callback)
    #Prevents the termination of the service.
    rospy.spin()
    
if __name__ == '__main__':
    main()
