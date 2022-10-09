#!/usr/bin/env python
import rospy
import actionlib
from math import sqrt,atan2,cos
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import time,sleep
import matplotlib.pyplot as plt

class Balancing:
	def __init__ (self):
		self.pose_bot=[0,0,0] #x,y,theta of the bot
		self.vel_bot=0 #current velocity of the bot
		self.omega_bot=0; #current angular velocity of the bot
		self.pose_left_bot=[0,0,0]
		self.vel_left_bot=0
		self.pose_right_bot=[0,0,0]
		self.vel_right_bot=0		

		self.rate=rospy.Rate(50) #argument here is frequncy in Hz

		#subscribers
		rospy.Subscriber('/odom', Odometry, self.pose_callback) 
		rospy.Subscriber('/left_odom', Odometry, self.callback_left_odom) 
		rospy.Subscriber('/right_odom', Odometry, self.callback_right_odom) 

		#publishers
		self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

		#plotting
		self.waypoints=[[],[]]
		self.time_elapsed=[]
		self.start_time=0

		#velocity obstacle params:
		self.ANG_MAX=0.8
		self.VEL_MAX=0.15
		self.epsilon=0.0004

	def callback_left_odom(self,msg):
		self.pose_left_bot[0]=msg.pose.pose.position.x
		self.pose_left_bot[1]=msg.pose.pose.position.y
		self.vel_left_bot=msg.twist.twist.linear.x
		(roll,pitch,self.pose_left_bot[2])=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
		# print('left_bot:', self.pose_left_bot,'-----------------------------------------')

	def callback_right_odom(self,msg):
		self.pose_right_bot[0]=msg.pose.pose.position.x
		self.pose_right_bot[1]=msg.pose.pose.position.y
		self.vel_right_bot=msg.twist.twist.linear.x
		(roll,pitch,self.pose_right_bot[2])=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

	def pose_callback(self,msg):
		self.pose_bot[0]=msg.pose.pose.position.x
		self.pose_bot[1]=msg.pose.pose.position.y
		self.vel_bot=msg.twist.twist.linear.x
		self.omega_bot=msg.twist.twist.angular.z
		(roll,pitch,self.pose_bot[2])=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
		# print('Recorede yaw is :',self.pose_bot[2])
		self.time_elapsed.append(time()-self.start_time)
		self.waypoints[0].append(self.pose_bot[0])
		self.waypoints[1].append(self.pose_bot[1])

	def velocity_convert(self,theta, vel_x, vel_y):

	    v_lin =  min(max(vel_x, -self.VEL_MAX), self.VEL_MAX)
	    v_ang = 0
	    print(v_lin,v_ang)
	    return v_lin, v_ang

	def distance(self,a,b):
		# print(sqrt((a[0]-b[0])**2+(a[1]-b[1])**2))
		return sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

	def moveToNextStep(self,vel):
		(v_lin,v_ang)=self.velocity_convert(self.pose_bot[2],vel[0],vel[1])
		vel_msg=Twist()
		vel_msg.linear.x=v_lin
		vel_msg.angular.z=v_ang
		self.pub_vel.publish(vel_msg)

	def balancing_algo(self):
		print('Starting velocity obstacles Algo to move to the goal')
		self.start_time=time()
		cnt=0
		while (True):
			u=self.distance(self.pose_bot,self.pose_right_bot)-self.distance(self.pose_bot,self.pose_left_bot)
			self.moveToNextStep([u,0.0])
			if (self.vel_bot<self.epsilon and self.vel_right_bot<self.epsilon and self.vel_left_bot<self.epsilon and cnt>1000):
				break
			cnt+=1
			self.rate.sleep()
		return 1



if __name__ == '__main__':
	rospy.init_node('balancing')
	rospy.loginfo('balancing node has been created')
	Solver=Balancing()
	Solver.balancing_algo()
	rospy.loginfo('Yay Common Consencus acheived')
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









