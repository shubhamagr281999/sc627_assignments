#!/usr/bin/env python
import rospy
import actionlib
from math import sqrt,atan2,cos,asin,pi,sin,acos,asin
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
		self.obstacle_cone_angle=[]
		self.obstacle_cone_origin=[]
		self.obstacle_cone_axis=[]

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
		self.VEL_MAX=0.15
		self.lin_threshold=0.4
		self.obstacle_dia=0.15
		self.bot_dia=0.15
		self.alpha=0.7
		self.max_angle_dev=0.175
		self.max_vel_dev=0.05

	def Obstacle_update_callback(self,msg):
		n=len(msg.obstacles)			
		self.obstacle_cone_angle=np.zeros(n)
		self.obstacle_cone_origin=np.zeros([n,2])
		self.obstacle_cone_axis=np.zeros([n,2])
		cnt=0
		for i in msg.obstacles:
			self.obstacle_cone_angle[cnt]=asin((self.obstacle_dia+self.bot_dia)/(self.distance(self.pose,[i.pose_x,i.pose_y])))
			self.obstacle_cone_origin[cnt]=[i.vel_x,i.vel_y]
			self.obstacle_cone_axis[cnt]=self.normalise([i.pose_x-self.pose[0],i.pose_y-self.pose[1]])
			cnt+=1

	def normalise(self,vector):
		k=sqrt(vector[0]**2+vector[1]**2)
		return [vector[0]/k,vector[1]/k]

	def cos_from_dot(self,a,b):
		a_mag=sqrt(a[0]**2+a[1]**2)
		b_mag=sqrt(b[0]**2+b[1]**2)
		a=[a[0]/a_mag,a[1]/a_mag]
		b=[b[0]/b_mag,b[1]/b_mag]
		return acos(a[0]*b[0]+b[1]*a[1])

	def pose_callback(self,msg):
		self.pose[0]=msg.pose.pose.position.x
		self.pose[1]=msg.pose.pose.position.y
		self.vel=msg.twist.twist.linear.x
		self.omega=msg.twist.twist.angular.z
		(_,_,self.pose[2])=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
		# print('Recorede yaw is :',self.pose[2])
		self.f.write('{:.5f} , {:.5f} \n'.format(self.pose[0],self.pose[1]))
		self.time_elapsed.append(time()-self.start_time)
		self.waypoints[0].append(self.pose[0])
		self.waypoints[1].append(self.pose[1])

	def velocity_convert(self,theta, vel_x, vel_y):
	    gain_ang = 8 #to be tuned
	    
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

	def check_not_in_collision_cone(self,vel):
		cnt=0
		for i in self.obstacle_cone_origin:
			# print(self.obstacle_cone_axis)
			# print(self.obstacle_cone_origin)
			# print(self.obstacle_cone_angle)
			if(self.cos_from_dot(self.obstacle_cone_axis[cnt],[vel[0]-i[0],vel[1]-i[1]])<self.obstacle_cone_angle[cnt]):
				return False
			cnt+=1
		return True

	def check_in_max_theta_dev(self,vel):
		ang = atan2(vel[1], vel[0])
		return abs(ang-self.pose[2])<self.max_angle_dev

	def dist_to_edge(self,angle):
		if (angle>=-pi/4 and angle<=pi/4):
			return self.max_vel_dev/cos(angle)
		elif(angle>=pi/4 and angle<=3*pi/4):
			return self.max_vel_dev/sin(angle)
		elif (angle>=3*pi/4 and angle<=5*pi/4):
			return self.max_vel_dev/cos(pi-angle)
		else:
			return self.max_vel_dev/cos(3*pi/2-angle)

	def v_max_check(self,vel):
		return sqrt(vel[0]**2+vel[1]**2)<self.VEL_MAX

	def direction_to_goal(self):
		return atan2(self.goal[1]-self.pose[1],self.goal[0]-self.pose[0])

	def choose_vel(self):
		direction_to_go=self.direction_to_goal()
		print('Direction to go:',direction_to_go,'----------------------------------------------------------------------------------')
		for i in range(35):
			scan_direction=direction_to_go + self.alpha*i/34	
			dist=self.dist_to_edge(scan_direction)
			print('scan_direction:',scan_direction)
			for j in range(6):
				vel=[self.vel*cos(self.pose[2])+cos(scan_direction)*dist*(5-j)/5,self.vel*sin(self.pose[2])+sin(scan_direction)*dist*(5-j)/5]
				collision_free=self.check_not_in_collision_cone(vel)
				Within_v_max=self.v_max_check(vel)
				within_angle_max=self.check_in_max_theta_dev(vel)
				print('vel:',vel,' | collision_free:',collision_free,' | Within_v_max:',Within_v_max,' | within_angle_max:',within_angle_max)
				if(collision_free and Within_v_max and within_angle_max):
					print('--------------i:',i,'| -------------------------------j:',j)
					return vel

			scan_direction=direction_to_go - self.alpha*i/34
			print('scan_direction:',scan_direction)	
			dist=self.dist_to_edge(scan_direction)
			for j in range(6):
				vel=[self.vel*cos(self.pose[2])+cos(scan_direction)*dist*(5-j)/5,self.vel*sin(self.pose[2])+sin(scan_direction)*dist*(5-j)/5]
				collision_free=self.check_not_in_collision_cone(vel)
				Within_v_max=self.v_max_check(vel)
				within_angle_max=self.check_in_max_theta_dev(vel)
				print('vel:',vel,' | collision_free:',collision_free,' | Within_v_max:',Within_v_max,' | within_angle_max:',within_angle_max)
				if(collision_free and Within_v_max and within_angle_max):
					print('--------------i:',i,'| -------------------------------j:',j)
					return vel
		print('Didnt find any good point to go to. Bot surrounded by obstacle. Not possible to avoid them while moving to goal.')
		return [0,0]
		
	def go_to_goal(self):
		

	def volcityObstacle_based_planer(self):
		print('Starting velocity obstacles Algo to move to the goal')
		self.start_time=time()
		while (self.distance(self.pose,self.goal)>self.lin_threshold):
			vel=self.choose_vel()
			print('vel',vel,'--------------------------------------------------------------')
			self.moveToNextStep(vel)
			self.rate.sleep()
		self.go_to_goal()
		print('YAY Goal reached !!!')
		return 1



if __name__ == '__main__':
	rospy.init_node('collision_avoidance')
	rospy.loginfo('collision_avoidance node has been created')
	Solver=Collision_avoidance()
	# print(Solver.dist_to_edge(0))
	# print(Solver.dist_to_edge(pi/2))
	# print(Solver.dist_to_edge(pi))
	# print(Solver.dist_to_edge(3*pi/2))
	# print(Solver.dist_to_edge(3*pi/4))
	# print(Solver.dist_to_edge(5*pi/4))
	Solver.volcityObstacle_based_planer()
	rospy.loginfo('The way points are saved in output.txt. Ploting the same now')
	# Solver.f.close()
	# plt.plot(Solver.waypoints[0],Solver.waypoints[1])
	# plt.title('Path taken by the bot')
	# plt.xlabel('X')
	# plt.ylabel('Y')
	# plt.show()
	# plt.plot(Solver.time_elapsed,Solver.waypoints[0])
	# plt.title('Time vs X position of bot')
	# plt.ylabel('pos_x')
	# plt.xlabel('time in seconds	')
	# plt.show()









