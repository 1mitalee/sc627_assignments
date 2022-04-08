#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  5 20:32:32 2022

@author: bhairaveeoza

"""
from helper import distancebetweenpoints
from helper import computeLineThroughTwoPoints
from helper import computeDistancePointToLine
from helper import computeDistancePointToSegment
from helper import computeDistancePointToPolygon
from helper import computeTangentVectorToPolygon
from Bug1 import BUG1

Start=(0,0)
Goal= (5,3)
step_size= 0.1
P1= [[1,2],[1,0],[3,0]]
P2= [[2,3 ], [4,1], [5,2]]
Q=[P1, P2]

BUG1(Start, Goal, step_size, Q)
print("done")

"""
p1= [2, 0]
p2= [5, 0]

q=[-1, 1]

#P= [[-1, -1], [-1, 1], [1, 1], [1, -1]]
P=[[0,0], [1,0], [0,1]]
#k=computeLineThroughTwoPoints(p1, p2)
#m=computeDistancePointToLine(q, p1, p2)

#l= computeDistancePointToSegment(q, p1, p2)
#print(l)

#print(k)
#print(m)

m= computeDistancePointToPolygon(P, q)
n=computeTangentVectorToPolygon(P, q)
print(m)
print(n)"""

