#!/usr/bin/env python
from math import sqrt
import numpy as np

def computeLineThroughTwoPoints(P1,P2):
	b_=P2[0]-P1[0]
	a_=P1[1]-P2[1]
	c_=P1[1]*(P1[0]-P2[0])+P1[0]*(P2[1]-P1[1])
	norm=sqrt(a_**2+b_**2)
	return [a_/norm,b_/norm,c_/norm]

def computeDistancePointToLine(q,P1,P2):
	line=computeLineThroughTwoPoints(P1,P2)
	return abs(line[0]*q[0]+line[1]*q[1]+line[2])

def computeDistancePointToSegment(q,P1,P2):
	line=computeLineThroughTwoPoints(P1,P2)
	d=line[0]*q[0]+line[1]*q[1]+line[2]
	k_= [q[0]-d*line[0],q[1]-d*line[1]]
	# print(k_)
	if((k_[0]-P1[0])*(k_[0]-P2[0])>=0 and (k_[1]-P1[1])*(k_[1]-P2[1])>=0):
		if(abs(k_[0]-P2[0])>=abs(k_[0]-P1[0]) and abs(k_[1]-P2[1])>=abs(k_[1]-P1[1])):
			return sqrt((q[0]-P1[0])**2+(q[1]-P1[1])**2)
		else:
			return sqrt((q[0]-P2[0])**2+(q[1]-P2[1])**2)
	else:
		# print('here')
		return abs(d)

def computeDistancePointToPolygon(q,P):
	n=len(P)
	distances=[]
	for i in range(n-1):
		distances.append(computeDistancePointToSegment(q,P[i],P[(i+1)]))	
	distances=np.array(distances)
	return np.amin(distances)

def computeTangentVectorToPolygon(q,P):
	n=len(P)
	distances=[]
	for i in range(n-1):
		distances.append(computeDistancePointToSegment(q,P[i],P[(i+1)]))
	distances=np.array(distances)
	dist=np.amin(distances)
	# print(distances)
	a=np.where(distances==dist)
	if(len(a[0])>1):
		#it is a vertex (logic is that the same distance would be seen for two line sharing same vertex)
		if(a[0][0]==0 and a[0][1]!=1): #this is the case when its b/w first and last line in which case normal logic will fail
			x=q[0]-P[0][0]
			y=q[1]-P[0][1]
		else:
			x=q[0]-P[a[0][1]][0]
			y=q[1]-P[a[0][1]][1]

		norm=sqrt(x**2+y**2)
		return [-y/norm,x/norm] #check the sign for anticlockwise here
		
	elif(len(a[0])>0):
		#it is a line
		x=P[a[0][0]+1][0]-P[a[0][0]][0]
		y=P[a[0][0]+1][1]-P[a[0][0]][1]
		norm=sqrt(x**2+y**2)
		return [x/norm,y/norm]

	else:
		print('error check logic')







