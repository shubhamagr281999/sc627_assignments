#!/usr/bin/env python
import rospy
import actionlib
from math import sqrt,atan2
import numpy as np
from helper import *
from sc627_helper.msg import MoveXYAction,MoveXYGoal, MoveXYResult
from time import time
import matplotlib.pyplot as plt

class Potential_planner:
	def __init__ (self):
		self.start=[]
		self.next_pose=[0,0]
		self.goal=[]
		self.obstacles=[]
		# self.rate=rospy.Rate(1)

		#Initialize client
		self.client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
		self.client.wait_for_server()
		self.result=MoveXYResult()
		self.result.pose_final.x=0
		self.result.pose_final.y=0
		self.theta=0

		#reading input file, assuming that the polygon vertices are in anticlockwise direction
		self.ReadFile()
		# print(computeDistancePointToPolygon([2.91,2.35],self.obstacles[1]))
		# print(computeTangentVectorToPolygon([2.91,1.35],self.obstacles[1]))
		self.f=open('output.txt','w')
		self.f.write('{:.5f} , {:.5f} \n'.format(self.start[0],self.start[1]))

		#plotting
		self.waypoints=[[self.start[0]],[self.start[1]]]
		self.distance_to_goal=[self.distance(self.start,self.goal)]
		self.time_elapsed=[0]
		self.start_time=0

		#potential function params:
		self.chi=0.8
		self.Dstar=2
		self.neta=0.8
		self.Qstar=2

	def ReadFile(self):
		with open('input.txt','r') as f:
			lines=f.readlines()

			#reading start
			line1=lines[0].split(',')
			self.start=[float(line1[0]),float(line1[1])]

			#reading goal
			line2=lines[1].split(',')
			self.goal=[float(line2[0]),float(line2[1])]

			#reading step size
			self.step_size=float(lines[2])

			#reading obstacles assuming that the obstacle vertx are in anticlockwise sense
			obs=[]
			for i in range(4,len(lines)):
				if(lines[i]=='\n'):
					obs.append(obs[0]) #adding the first  vertex again so that the line4 can be created required in logic
					self.obstacles.append(obs)
					obs=[]
				else:	
					temp=lines[i].split(',')
					obs.append([float(temp[0]),float(temp[1])])
			obs.append(obs[0])
			self.obstacles.append(obs)
		# print('Sucessfully read the input file.')
		# print(self.obstacles)
		# print(len(self.obstacles))
		# print(self.step_size)
		# print(self.start)

	def distance(self,a,b):
		# print(sqrt((a[0]-b[0])**2+(a[1]-b[1])**2))
		return sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

	def moveToNextStep(self,tangent):
		msg=MoveXYGoal()
		msg.pose_dest.x=self.next_pose[0]
		msg.pose_dest.y=self.next_pose[1]
		msg.pose_dest.theta=tangent
		print(self.next_pose[0],self.next_pose[1],tangent)

		self.client.send_goal(msg)
		self.client.wait_for_result()
		self.result=self.client.get_result()
		self.theta=self.result.pose_final.theta
		self.start=[self.result.pose_final.x,self.result.pose_final.y]

		# self.start=[self.next_pose[0],self.next_pose[1]]

		self.f.write('{:.5f} , {:.5f} \n'.format(self.start[0],self.start[1]))
		# self.start=self.next_pose
		self.distance_to_goal.append(self.distance(self.start,self.goal))
		self.time_elapsed.append(time()-self.start_time)
		self.waypoints[0].append(self.start[0])
		self.waypoints[1].append(self.start[1])

	def obstaclesPotential(self):
		n=len(self.obstacles)
		gradient=np.zeros(2)
		for i in range(n):
			distance=computeDistancePointToPolygon(self.start,self.obstacles[i])
			if(distance<self.Qstar):
				c=computeClosetPointToPolygon(self.start,self.obstacles[i])
				gradient = gradient + self.neta*(1/self.Qstar-1/distance)*(1/distance)**3*np.array([self.start[0]-c[0],self.start[1]-c[1]])
		return gradient

	def goalPotential(self):
		distance=self.distance(self.start,self.goal)
		if(distance>self.Dstar):
			return self.chi*self.Dstar*(1/distance)*np.array([self.start[0]-self.goal[0],self.start[1]-self.goal[1]])
		else:
			return self.chi*np.array([self.start[0]-self.goal[0],self.start[1]-self.goal[1]])


	def potential_based_planer(self):
		print('Starting Potential function Algo to move to the goal')
		self.start_time=time()
		while (self.distance(self.start,self.goal)>self.step_size):
			direction_to_go=-1*self.obstaclesPotential() + -1*self.goalPotential()
			print(np.linalg.norm(direction_to_go))
			if(np.linalg.norm(direction_to_go)<0.32):
				print('reached a minima')
				break
			# direction_to_go=direction_to_go/(np.linalg.norm(direction_to_go))
			self.next_pose[0]=self.start[0]+direction_to_go[0]*self.step_size
			self.next_pose[1]=self.start[1]+direction_to_go[1]*self.step_size
			self.moveToNextStep(atan2(direction_to_go[1],direction_to_go[0]))
		return 1



if __name__ == '__main__':
	rospy.init_node('bug_algo')
	rospy.loginfo('bug algorithm solver nide has been created')
	pfSolver=Potential_planner()
	pfSolver.potential_based_planer()
	pfSolver.f.close()
	rospy.loginfo('The way points are saved in output.txt. Closing the node')
	plt.plot(pfSolver.waypoints[0],pfSolver.waypoints[1])
	plt.title('Path taken by the bot')
	plt.xlabel('X')
	plt.ylabel('Y')
	plt.show()
	plt.plot(pfSolver.time_elapsed,pfSolver.distance_to_goal)
	plt.title('Time vs distance_to_goal')
	plt.ylabel('distance_to_goal')
	plt.xlabel('time in seconds	')
	plt.show()









