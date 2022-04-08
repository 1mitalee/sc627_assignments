#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
from helper import *
import statistics
import matplotlib.pyplot as plt

def BUG0(Start, Goal, StepSize, Obstaclelist):
    #Obstaclelist is an array of polygons, each polygon is an array of points
    current_position= Start
    Path= np.array([Start])
    Status= np.array([[current_position[0], current_position[1], "towardsGoal","m"]])
    N=len(Obstaclelist)
    Obstacle_centroids= np.zeros((N, 2))
    for i in range(N):
        Obstacle_centroids[i]= [0,0]
        for k in range(len(Obstaclelist[i])):
            Obstacle_centroids[i]=Obstacle_centroids[i] + Obstaclelist[i][k]
            
        Obstacle_centroids[i]= Obstacle_centroids[i]/len(Obstaclelist[i])
        
    #print(Obstacle_centroids)
    
    # M1 is to keep track of the number of times the while loop is run
    M1= 0
    while (distancebetweenpoints(Goal, current_position)>StepSize ):
        
        Dist2Obst= np.zeros(N)
        for i in range(N):
            Dist2Obst[i]= computeDistancePointToPolygon(Obstaclelist[i],current_position)[0]
    
        
        min_dist= np.min(Dist2Obst)
        ClosestObst_index= 0
        for i in range(N):
            if(Dist2Obst[ClosestObst_index] > Dist2Obst[i]):
                ClosestObst_index=i
        #print(Dist2Obst, ClosestObst_index )
        #print(Dist2Obst)
        #print(ClosestObst_index)
        
        #Assuming only 1 obstacle is closer than step size at a time
        if (min_dist> StepSize):
            #print("Towardsgoal")
            u1= ((Goal[0]-current_position[0])/distancebetweenpoints(Goal, current_position)) * StepSize
            u2= ((Goal[1]-current_position[1])/distancebetweenpoints(Goal, current_position)) * StepSize
            u=[u1, u2]
            current_position= [current_position[0] + (u[0]), current_position[1] + (u[1])]
            Path= np.append(Path, [current_position])
            Status= np.append(Status,[current_position[0], current_position[1], "towardsGoal", "M"])
            State= "success"
            
            
        else:
            State= "Failed"
            print("failed")
            break
            
            
        M1= M1 + 1
    M= len(Path)
    #print(M)
    M=int(M/2)
    x_coor= np.array([0])  
    y_coor= np.array([0]) 
    for i in range(M):
        x_coor= np.append(x_coor, Path[2*i])
        y_coor= np.append(y_coor, Path[2*i + 1])
    path= np.zeros((M,2))
    for i in range(M):
        path[i][0]= x_coor[i]
        path[i][1]=y_coor[i]
        
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(x_coor, y_coor, color='lightblue', linewidth=3)
    
    
    path= np.zeros((M,2))
    for i in range(M):
        path[i][0]= x_coor[i]
        path[i][1]=y_coor[i]
    
    return State
    
    # print(path)
    

inFile = open("input.txt", "r")
temp_list = []
read_list = []
for line in inFile.readlines():
    if line == '\n':
        read_list.append(temp_list)
        temp_list = []

    else:
        temp = line.strip().split(",")
        temp = [float(i) for i in temp]
        temp_list.append(list(map(float, temp)))
read_list.append(temp_list)

start = np.array(read_list[0][0])
goal = np.array(read_list[0][1])
step_size = read_list[0][2][0]
read_list.pop(0)
obstaclesList = np.array(read_list)

State=BUG0(start, goal, step_size, obstaclesList)
outFile = open("output_base.txt", "w")

outFile.write(str(State)+ "\n")

    


