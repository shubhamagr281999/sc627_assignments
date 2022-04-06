#!/usr/bin/env python
import rospy
import actionlib
from math import sqrt,atan2,cos
import numpy as np
from sc627_helper.msg import ObsData
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import time,sleep
import matplotlib.pyplot as plt

class Collision_avoidance:
	def __init__ (self):
		self.pose=[0,0,0] #x,y,theta of the bot
		self.vel=0; #current velocity of the bot
		self.omega=0; #current angular velocity of the bot
		self.goal=[5,0]
		self.obstacle_pose=[]
		self.obstacle_vel=[]

		self.rate=rospy.Rate(50) #argument here is frequncy in Hz

		#publishers
		self.vel_pub=rospy.Publisher('/bot_1/cmd_vel',Twist,queue_size=1)

		#subscribers
		self.obstacles_sub=rospy.Subscriber('/obs_data',ObsData,self.Obstacle_update_callback,queue_size=1)
		self.odom_sub=rospy.Subscriber('/bot_1/odom',Odometry,self.pose_callback,queue_size=1)


		#writing path to a text file
		self.f=open('output.txt','w')
		self.f.write('{:.5f} , {:.5f} \n'.format(self.pose[0],self.pose[1]))

		#plotting
		self.waypoints=[[self.pose[0]],[self.pose[1]]]
		self.time_elapsed=[0]
		self.start_time=0

		#velocity obstacle params:
		self.ANG_MAX=0.8
		self.VEL_MAX=2
		self.lin_threshold=0.1

	def Obstacle_update_callback(self,msg):
		self.obstacle_pose=[]
		self.obstacle_vel=[]
		for i in msg.obstacles:
			self.obstacle_pose.append([i.pose_x,i.pose_y])
			self.obstacle_vel.append([i.vel_x,i.vel_y])
		# print('-----------------POSE-----------------')
		# print(self.obstacle_pose)
		# print('------------------vel------------------')
		# print(self.obstacle_vel)

	def pose_callback(self,msg):
		self.pose[0]=msg.pose.pose.position.x
		self.pose[1]=msg.pose.pose.position.y
		self.vel=msg.twist.twist.linear.x
		self.omega=msg.twist.twist.angular.z
		(roll,pitch,self.pose[2])=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
		# print('Recorede yaw is :',self.pose[2])
		self.f.write('{:.5f} , {:.5f} \n'.format(self.pose[0],self.pose[1]))
		self.time_elapsed.append(time()-self.start_time)
		self.waypoints[0].append(self.pose[0])
		self.waypoints[1].append(self.pose[1])

	def velocity_convert(self,theta, vel_x, vel_y):
	    '''
	    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
	    Velocity vector (vel_x, vel_y)
	    '''
	    gain_ang = 4 #to be tuned
	    
	    ang = atan2(vel_y, vel_x)
	    
	    ang_err = min(max(ang - theta, -self.ANG_MAX), self.ANG_MAX)
	    v_lin =  min(max(cos(ang_err) * sqrt(vel_x ** 2 + vel_y ** 2), -self.VEL_MAX), self.VEL_MAX)
	    v_ang = gain_ang * ang_err
	    return v_lin, v_ang

	def distance(self,a,b):
		# print(sqrt((a[0]-b[0])**2+(a[1]-b[1])**2))
		return sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

	def moveToNextStep(self,vel):
		(v_lin,v_ang)=self.velocity_convert(self.pose[2],vel[0],vel[1])
		vel_msg=Twist()
		vel_msg.linear.x=v_lin
		vel_msg.angular.z=v_ang
		self.vel_pub.publish(vel_msg)

	def volcityObstacle_based_planer(self):
		print('Starting velocity obstacles Algo to move to the goal')
		self.start_time=time()
		while (self.distance(self.pose,self.goal)>self.lin_threshold):
			self.rate.sleep()
		return 1



if __name__ == '__main__':
	rospy.init_node('collision_avoidance')
	rospy.loginfo('collision_avoidance node has been created')
	Solver=Collision_avoidance()
	Solver.volcityObstacle_based_planer()
	rospy.loginfo('The way points are saved in output.txt. Ploting the same now')
	Solver.f.close()
	plt.plot(Solver.waypoints[0],Solver.waypoints[1])
	plt.title('Path taken by the bot')
	plt.xlabel('X')
	plt.ylabel('Y')
	plt.show()
	plt.plot(Solver.time_elapsed,Solver.waypoints[0])
	plt.title('Time vs X position of bot')
	plt.ylabel('pos_x')
	plt.xlabel('time in seconds	')
	plt.show()









