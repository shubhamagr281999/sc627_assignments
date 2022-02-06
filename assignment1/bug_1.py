#!/usr/bin/env python
import rospy
from math import sqrt
import numpy as np
from helper import *
from sc627_helper.msg import MoveXYActionGoal,MoveXYActionResult
from time import sleep
class bugAlgorithm:
	def __init__ (self):
		self.start=[]
		self.next_pose=[]
		self.goal=[]
		self.obstacles=[]
		self.rate=rospy.Rate(1)

		#reading input file, assuming that the polygon vertices are in anticlockwise direction
		self.ReadFile()
		# print(computeDistancePointToPolygon([2.91,2.35],self.obstacles[1]))
		# print(computeTangentVectorToPolygon([2.91,1.35],self.obstacles[1]))
		self.f=open('output_1.txt','w')
		self.f.write('{:.5f} , {:.5f} \n'.format(self.start[0],self.start[1]))

		#publisher
		self.goal_pub=rospy.Publisher('/move_xy/goal',MoveXYActionGoal,queue_size=1)

		#subscriber
		self.odom_subscriber=rospy.Subscriber('/move_xy/result',MoveXYActionResult,self.odom_callback)

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

	def odom_callback(self,msg):
		self.start=[msg.result.pose_final.x,msg.result.pose_final.y]
		print('curent pose:',self.start)
	def distance(self,a,b):
		# print(sqrt((a[0]-b[0])**2+(a[1]-b[1])**2))
		return sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

	def obstaclesCheck(self):
		n=len(self.obstacles)
		for i in range(n):
			distance=computeDistancePointToPolygon(self.start,self.obstacles[i])
			if(distance<self.step_size):
				return True,computeTangentVectorToPolygon(self.start,self.obstacles[i]),i
		return False,-1,-1

	def moveOneStepTogoal(self):
		goal_direction=[self.goal[0]-self.start[0],self.goal[1]-self.start[1]]
		norm=sqrt(goal_direction[0]**2 + goal_direction[1]**2)
		self.next_pose=[self.start[0]+goal_direction[0]*self.step_size/norm, self.start[1]+goal_direction[1]*self.step_size/norm]
		print(self.next_pose)
		self.moveToNextStep()

	def moveToNextStep(self):
		msg=MoveXYActionGoal()
		msg.goal.pose_dest.x=self.next_pose[0]
		msg.goal.pose_dest.y=self.next_pose[1]
		self.f.write('{:.5f} , {:.5f} \n'.format(self.next_pose[0],self.next_pose[1]))
		# self.goal_pub.publish(msg)
		self.start=self.next_pose


	def bug_base(self):
		print('Starting BugBase Algo to move to the goal')
		while (self.distance(self.start,self.goal)>self.step_size):
			hindrance,tangent,obstacleNum = self.obstaclesCheck()			
			if(hindrance):
				print('encountered a obstacles. Cannot find a path. Failed :(')
				return -1
			else:
				self.moveOneStepTogoal()
			sleep(0.1)

		print('Goal reached :)')
		return 1

	def bug_1(self):
		print('Starting Bug1 Algo to move to the goal')
		while (self.distance(self.start,self.goal)>self.step_size):
			hindrance,tangent,obstacleNum = self.obstaclesCheck()			
			if(hindrance):
				phit=computeClosetPointToPolygon(self.start,self.obstacles[obstacleNum])
				pcurrent=phit
				pRelase=phit
				min_dist=self.distance(phit,self.goal)
				
				while(np.dot([phit[0]-pcurrent[0],phit[1]-pcurrent[1]],tangent)<=0 or self.distance(phit,pcurrent)>self.step_size):
					#need to go around the obstacle
					self.next_pose=[self.start[0]+tangent[0]*self.step_size,self.start[1]+tangent[1]*self.step_size] #moving along the obstacle
					self.moveToNextStep()
					tangent=computeTangentVectorToPolygon(self.start,self.obstacles[obstacleNum])
					# print(self.next_pose,tangent)
					# sleep(3)
					pcurrent=computeClosetPointToPolygon(self.start,self.obstacles[obstacleNum])	
					dist_to_goal=self.distance(pcurrent,self.goal)
					if(dist_to_goal<min_dist):
						pRelase=pcurrent
						min_dist=dist_to_goal
				print('encountered a obstacles. circled it to find the P release point. Now will move to P relase point')

				tangent=computeTangentVectorToPolygon(self.start,self.obstacles[obstacleNum])
				pcurrent=computeClosetPointToPolygon(self.start,self.obstacles[obstacleNum])
				while(self.distance(pRelase,pcurrent)>self.step_size):
					self.next_pose=[self.start[0]+tangent[0]*self.step_size,self.start[1]+tangent[1]*self.step_size] #moving along the obstacle
					self.moveToNextStep()
					tangent=computeTangentVectorToPolygon(self.start,self.obstacles[obstacleNum])
					pcurrent=computeClosetPointToPolygon(self.start,self.obstacles[obstacleNum])				
				print('clered a obstacle')
			else:
				self.moveOneStepTogoal()

		print('Goal reached :)')
		return 1

	def bug_0(self):
		print('Starting Bug0 Algo to move to the goal')
		goal_visibility=0
		while (self.distance(self.start,self.goal)>self.step_size):
			hindrance,tangent,obstacleNum = self.obstaclesCheck()			
			if(hindrance):
				goal_visibility=np.cross([self.goal[0]-self.start[0],self.goal[1]-self.start[1]],tangent)
				# print(goal_visibility,tangent)
				while (goal_visibility<0):					
					self.next_pose=[self.start[0]+tangent[0]*self.step_size,self.start[1]+tangent[1]*self.step_size] #moving along the obstacle
					self.moveToNextStep()
					# print(self.start,obstacleNum,tangent)
					
					tangent=computeTangentVectorToPolygon(self.start,self.obstacles[obstacleNum])
					goal_visibility=np.cross([self.goal[0]-self.start[0],self.goal[1]-self.start[1]],tangent)
					sleep(0.1)
				print('cleared a obstacles. The gaol is visible again.')
			
			if(goal_visibility>=0 and (not hindrance)):
				self.moveOneStepTogoal()
			sleep(0.1)

		print('Goal reached :)')
		return 1

if __name__ == '__main__':
	rospy.init_node('bug_algo')
	rospy.loginfo('bug algorithm solver nide has been created')
	bugSolver=bugAlgorithm()
	bugSolver.bug_1()
	bugSolver.f.close()
	rospy.loginfo('The way points are saved in output.txt. Closing the node')







