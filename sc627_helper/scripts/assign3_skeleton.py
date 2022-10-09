#!/usr/bin/env python3

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

ANG_MAX = math.pi/18
VEL_MAX = 0.15

def velocity_convert(x, y, theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 1 #modify if necessary
    
    ang = math.atan2(vel_y, vel_x)
    if ang < 0:
        ang += 2 * math.pi
    
    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang


def callback_obs(data):
    '''
    Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
    '''

    print(data)
    pass


def callback_odom(data):
    '''
    Get robot data
    '''

    print(data)
    pass


rospy.init_node('assign3_skeleton', anonymous = True)
rospy.Subscriber('/obs_data', ObsData, callback_obs) #topic name fixed
rospy.Subscriber('/bot_1/odom', Odometry, callback_odom) #topic name fixed

pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)
r = rospy.Rate(30)

while True: #replace with destination reached?
    #calculate collision cone below

    #calculate v_x, v_y as per either TG, MV, or ST strategy
    #Make sure your velocity vector is feasible (magnitude and direction)

    #convert velocity vector to linear and angular velocties using velocity_convert function given above

    #publish the velocities below
    vel_msg = Twist()
    # vel_msg.linear.x = v_lin
    # vel_msg.angular.z = v_ang
    pub_vel.publish(vel_msg)
    
    #store robot path with time stamps (data available in odom topic)

    r.sleep()




