#!/usr/bin/env python
from math import sqrt
import numpy as np
from helper import *

class bug_base:
	def __init__ (self):
		self.start=[]
		self.goal=[]
		self.obstacles=[]
		#reading input file, assuming that the polygon vertices are in anticlockwise direction
		self.ReadFile()
		# print(computeDistancePointToPolygon([2.91,2.35],self.obstacles[1]))
		# print(computeTangentVectorToPolygon([2.91,1.35],self.obstacles[1]))

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
		self.start=[self.start[0]+goal_direction[0]*self.step_size/norm, self.start[1]+goal_direction[1]*self.step_size/norm]
		# print(self.start)
		#write to output txt file

	def bug_algo(self):
		print('Starting BugBase Algo to move to the goal')
		goal_visibility=0
		while (self.distance(self.start,self.goal)>self.step_size):
			hindrance,tangent,obstacleNum = self.obstaclesCheck()			
			if(hindrance):
				goal_visibility=np.cross([self.goal[0]-self.start[0],self.goal[1]-self.start[1]],tangent)
				# print(goal_visibility,tangent)
				while (goal_visibility<0):					
					self.start=[self.start[0]+tangent[0]*self.step_size,self.start[1]+tangent[1]*self.step_size] #moving along the obstacle
					
					# print(self.start,obstacleNum,tangent)
					#write this to output txt file
					tangent=computeTangentVectorToPolygon(self.start,self.obstacles[obstacleNum])
					goal_visibility=np.cross([self.goal[0]-self.start[0],self.goal[1]-self.start[1]],tangent)
				print('cleared a obstacles. The gaol is visible again.')
			
			if(goal_visibility>=0 and (not hindrance)):
				self.moveOneStepTogoal()

		print('Goal reached :)')
		return 1


if __name__ == '__main__':
    bugSolver=bug_base()
    bugSolver.bug_algo()








