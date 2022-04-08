#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
from helper_2 import *
import matplotlib.pyplot as plt


def Compute_AttractivePotential(current_position, goal, para_m, para_d):
    D= distancebetweenpoints(current_position, goal)
    
    if (D <= para_d):
        potential= 0.5 * para_m * (D**2)
    else:
        potential= para_d*para_m* D -(0.5*para_m* (para_d**2))
        
    return potential

def Compute_RepulsivePotential_i(current_position, P, para_n, para_q):
    D= computeDistancePointToPolygon(current_position, P)
    if (D <= para_q):
        potential= 0.5 * para_n * (((1/D)-(1/para_q))**2)
    else:
        potential=0

def Gradient_Attractive(current_position, goal, para_m, para_d):
    D= distancebetweenpoints(current_position, goal)
    if (D <= para_d):
        gradient= para_m * (current_position-goal)
    else:
        gradient= para_d * para_m * (1/D)* (current_position-goal)
    return gradient

def Gradient_Repulsive_i(current_position, P, para_n, para_q):
    D= computeDistancePointToPolygon(P, current_position)[0]
    if (D <= para_q):
        gradient= para_n * ((1/para_q) - (1/D))* (1/(D**2))* gradientDistancePointToPolygon(P, current_position)
    else:
        gradient= np.array([0,0])
    return gradient

def Gradient_Repulsive(current_position, Q, para_n, para_d):
    m= len(Q)
    gradient= np.array([0, 0])
    for i in range(m):
        gradient= gradient+ Gradient_Repulsive_i(current_position, Q[i], para_n, para_q)
    
    return gradient
        

def Gradient(current_position, goal, stepsize, Q, para_m, para_d, para_n, para_q):
    
    gradient= Gradient_Attractive(current_position, goal, para_m, para_d) + Gradient_Repulsive(current_position, Q, para_n, para_d)
    
    return gradient

def Gradient_norm(current_position, goal, stepsize, Q, para_m, para_d, para_n, para_q):
    gradient= Gradient(current_position, goal, stepsize, Q, para_m, para_d, para_n, para_q)
    gradientnorm= math.sqrt(gradient[0]**2 + gradient[1]**2 )
    return gradientnorm
    
    

def potential_function_planner(start, goal, stepsize, Q, para_m, para_d, para_n, para_q):
    
    current_position=start
    path= np.array([start])
    i=0
    
    while (Gradient_norm(current_position, goal, stepsize, Q, para_m, para_d, para_n, para_q)>0.01 ):
        current_position= current_position - 0.1*(Gradient(current_position, goal, stepsize, Q, para_m, para_d, para_n, para_q)/Gradient_norm(current_position, goal, stepsize, Q, para_m, para_d, para_n, para_q))
        path= np.append(path, [current_position])
        i= i+1
    print("Destination Reached")
    
    
    M= len(path)
    #print(M)
    M=int(M/2)
    x_coor= np.array([0])  
    y_coor= np.array([0]) 
    for i in range(M):
        x_coor= np.append(x_coor, path[2*i])
        y_coor= np.append(y_coor, path[2*i + 1])
        
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.plot(x_coor, y_coor, color='lightblue', linewidth=3)
    path= np.zeros((M,2))
    for i in range(M):
        path[i][0]= x_coor[i]
        path[i][1]=y_coor[i]
    
    # print(path)
    
    
    return path
    
    

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

para_m= 0.8
para_d= 2
para_n= 0.8
para_q= 2

path= potential_function_planner(start, goal, step_size, obstaclesList, para_m, para_d, para_n, para_q)

outFile = open("output_1.txt", "w")
for element in path:
    outFile.write(str(element[0]) + ", " + str(element[1]) + "\n")
    
# Dist_goal= np.zeros(len(path))
# Index= np.zeros(len(path))

# for k in range(len(path)):
#     Dist_goal[k]= distancebetweenpoints(goal, path[k])
#     Index[k]= k * 0.001
    
# fig = plt.figure()
# plt.plot(Index, Dist_goal)


    

    


    
        
    
    
    

