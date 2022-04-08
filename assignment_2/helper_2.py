#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import math

# points are assumed to be 2 D arrays

def distancebetweenpoints(p1, p2):
    d = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    return d

def computeLineThroughTwoPoints(p1, p2):
    if (p1[0]== p2[0] and p1[1]== p2[1]):
        print("Invalid Input: multiple possible lines")
        a=0
        b=0
        c=0
    elif (p1[0]== p2[0]):
        a=1
        b=0
        c= - p1[0]
    elif (p1[1]== p2[1]):
        a=0
        b=1
        c= -p1[1]
    else:
        m= (p2[1]-p1[1])/(p2[0]-p1[0])
        c= p1[1] - (m* p1[0])
        a= m/math.sqrt(m**2 + 1)
        b= -1/math.sqrt(m**2 + 1)
        c= c/math.sqrt(m**2 + 1)
        
    return [a,b,c]

def computeDistancePointToLine(q, p1, p2):
    v = computeLineThroughTwoPoints(p1, p2)
    if (v==[0, 0, 0]):
        print("Invalid Input")
        d=0
    else:
        d= abs(q[0]*v[0] + q[1]*v[1]+ v[2])
        
    return d

def computeDistancePointToSegment(q, p1, p2):
    # define the point closest to q on the line defined by the 2 points
    v = computeLineThroughTwoPoints(p1, p2)
    a= v[0]
    b= v[1]
    c= v[2]
    
    q_x= q[0]
    q_y=q[1]
    
    m_x= b*(b*q_x - a*q_y)-a*c
    m_y= -a * (b*q_x - a*q_y) - b*c
    
    m= [m_x, m_y]
    
    d_p1= distancebetweenpoints(p1,m)
    d_p2= distancebetweenpoints(p2,m)
    d_p= distancebetweenpoints(p1,p2)
    
    
    if(d_p==(d_p1+ d_p2)):
        if(d_p1 == 0):
            d= distancebetweenpoints(q,m)
            w= 1
        elif(d_p2 ==0):
            d=distancebetweenpoints(q,m)
            w=2
        else:
            d= distancebetweenpoints(q,m)
            w=0
    else:
        if (d_p1<d_p2):
            d= distancebetweenpoints(q,p1)
            w= 1
        else:
            d= distancebetweenpoints(q,p2)
            w=2
            
    return [d, w]

def computeDistancePointToPolygon(P, q):
    # P is an array of n points, each point a 2D array
    n= len(P)
    M=np.zeros(n)
    
    for i in range(n-1):
        p1= P[i]
        p2= P[i+1]
        
        M[i]= computeDistancePointToSegment(q, p1, p2)[0]
        
    M[n-1]= computeDistancePointToSegment(q, P[n-1], P[0])[0]
    
    
    min_value= np.min(M)
    min_index= 0
    for i in range(n):
        if(M[min_index] > M[i]):
            min_index= i
            
    return(min_value, min_index)

def gradientDistancePointToPolygon(P, q):
    m= computeDistancePointToPolygon(P, q)[1]
    C= P[m]
    gradient= q - C
    gradient= gradient/(computeDistancePointToPolygon(P, q)[0])
    return gradient
    

def computeTangentVectorToPolygon(P, q):
    n= len(P)
    K= computeDistancePointToPolygon(P, q)
    i = K[1]

    if (i != n-1):
        p1= P[i]
        p2= P[i+1]
        if(i != 0):
            p3= P[i-1]
        elif(i ==0):
            p3= P[n-1]
    #P3 for checkig which side


    elif(i == n-1):
        p1= P[n-1]
        p2= P[0]
        p3= P[1]
    
    q2polygon=computeDistancePointToSegment(q, p1, p2)

    if(q2polygon[1]==0):
        #assuming convex polygons, so that all points can lie on one side
        v= computeLineThroughTwoPoints(p1, p2)
        a= v[0]
        b= v[1]
        c= v[2]
        value= a* p3[0] + b* p3[1] + c
        if(value > 0):
            u1= b
            u2= -a
            
        else:
            u1= -b
            u2= a
        
        u1= u1/ math.sqrt(u1**2 + u2**2)
        u2= u2/ math.sqrt(u1**2 + u2**2)

    elif(q2polygon[1]==1):
        u1= -(q[1]- p1[1])
        u2= q[0]-p1[0]
        u1= u1/ math.sqrt(u1**2 + u2**2)
        u2= u2/ math.sqrt(u1**2 + u2**2)

    elif(q2polygon[1]==2):
        u1= -(q[1]- p2[1])
        u2= q[0]-p2[0]
        u1= u1/ math.sqrt(u1**2 + u2**2)
        u2= u2/ math.sqrt(u1**2 + u2**2)  
    
    u=[u1, u2]
    return u

#function for finding unit vector from one point to another
def ComputeUnitVector(p1, p2):
    u1= (p1[0] - p2[0])/distancebetweenpoints(p1, p2)
    u2= (p1[1] - p2[1])/distancebetweenpoints(p1, p2)
    u=[u1, u2]
    return u

def DotProductArrays(v1, v2):
    m= v1[0]*v2[0] + v1[1]*v2[0]
    return m

# p1= np.array([1,1])
# p2= np.array([2,2])
# m= distancebetweenpoints(p1, p2)   
# print(m) 



    
    
 

"""
def computeTangentVectorToPolygon(P, q):
    n= len(P)
    K= computeDistancePointToPolygon(P, q)
    i= K[1]
    

    if (i != n-1):
        p1= P[i]
        p2= P[i+1]
    
        
    elif(i==n-1):
       p1= P[n-1]
       p2= P[0] 
    
    q2polygon=computeDistancePointToSegment(q, p1, p2)
    
    if(q2polygon[1]==0):
        #assuming convex polygons, so that all points lie on one side
        v=computeLineThroughTwoPoints(p1, p2)
        a= v[0]
        b= v[1]
        ADDD HERE
        
        
        
    elif(q2polygon[1]==1):
        u1= -(q[1]- p1[1])
        u2= q[0]-p1[0]
        #for anticlockwise
        u1= u1/ math.sqrt(u1**2 + u2**2)
        u2= u2/ math.sqrt(u1**2 + u2**2)
        
    elif(q2polygon[1]==2):
        u1= -(q[1]- p2[1])
        u2= q[0]-p2[0]
        #for anticlockwise
        u1= u1/ math.sqrt(u1**2 + u2**2)
        u2= u2/ math.sqrt(u1**2 + u2**2)
    u=[u1, u2]
    return u
    
 """   
        
    
    
    
        
    


    
        


