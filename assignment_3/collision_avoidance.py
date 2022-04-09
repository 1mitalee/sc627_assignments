#!/usr/bin/env python

import rospy
import actionlib
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import time
import numpy as np

class CollisionAvoidance():
    
    
    def __init__(self):
        self.start= np.array([0, 0])
        self.goal=[5.0, 0]
        self.diam= 0.15 
        self.vmax= 0.15
        self.angmax=math.pi/18
        
        self.v_discrete= 0.015
        self.ang_discrete= math.pi/180
        
        self.currentpose= [0.0, 0.0, 0.0]
        self.currentv= [0.0, 0.0]
        self.currentw= 0.0
        
        self.obs_x= np.ones(3)
        self.obs_y= np.ones(3)
        self.obs_vx=np.ones(3)
        self.obs_vy= np.ones(3)
        
        self.cone_angle=np.zeros(3)
        
        self.workspace= [[], []]
        self.freespace= [[], []]
        
        self.rate = rospy.Rate(30)
        
        self.start = 0
        self.time_abs=[]
        self.trajectory= [[], []]
        
        self.min_val= 2* math.pi
        self.min_vel= None
        
        self.epsilon= 0.4
        
        rospy.Subscriber('/obs_data', ObsData, self.callback_obs)
        rospy.Subscriber('/bot_1/odom', Odometry, self.callback_odom)
        
        self.pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)
        
    def velocity_convert(self, theta, vel_x, vel_y):
        '''
        Robot pose (x, y, theta)  Note - theta in (0, 2pi)
        Velocity vector (vel_x, vel_y)
        '''

        gain_ang = 3 #modify if necessary
        
        ang = math.atan2(vel_y, vel_x)
        if ang < 0:
            ang += 2 * math.pi
        
        ang_err = min(max(ang - theta, - self.angmax), self.angmax)

        v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), - self.vmax), self.vmax)
        v_ang = gain_ang * ang_err
        return v_lin, v_ang
    
    def callback_odom(self, msg):
        
        self.currentpose[0]= msg.pose.pose.position.x
        self.currentpose[1]= msg.pose.pose.position.y
        
        self.currentv[0] = msg.twist.twist.linear.x
        self.currentv[1]= msg.twist.twist.linear.y
        self.currentw= msg.twist.twist.angular.z
        
        (_,_,self.currentpose[2])=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        
        self.trajectory[0].append(self.currentpose[0])
        self.trajectory[1].append(self.currentpose[1])
        self.time_abs.append(time.time()- self.start)
    
    def callback_obs(self, msg):
        
        i=0
        for obstacle in msg.obstacles:
            self.obs_x[i]= obstacle.pose_x
            self.obs_y[i]= obstacle.pose_y
            self.obs_vx[i]= obstacle.vel_x
            self.obs_vy[i]=obstacle.vel_y
            i= i+1
            
    def generateworkspace(self):
        
        theta= self.currentpose[2]
        self.workspace=[[], []]
        
        for vel in np.linspace(0, self.vmax, (2*self.vmax/self.v_discrete)):
            for angle in np.linspace(theta-self.angmax, theta + self.angmax, (2*self.angmax/self.ang_discrete)):
                
                if vel !=0:
                    self.workspace[0].append(vel*np.cos(angle))
                    self.workspace[1].append(vel*np.sin(angle))
                    
            
    def generateCollision_i(self, i):
        
        Bot= [self.currentpose[0], self.currentpose[1]]
        Obstacle= [self.obs_x[i], self.obs_y[i]]
        distance= math.sqrt((Obstacle[0]-Bot[0])**2 + (Obstacle[1]-Bot[1])**2)
        
        if(2*self.diam > distance):
            return math.pi
        
        angle= np.arcsin(2*self.diam/distance)
        
        return angle
    
    def CollisionConeList(self):
        
        for i in range(len(self.obs_x)):
            self.cone_angle[i]= self.generateCollision_i(i)
            
    def anglebetweenvectors(self, v1, v2):
        norm1= math.sqrt((v1[0])**2 + (v1[1]) **2)
        norm2= math.sqrt((v2[0])**2 + (v2[1]) **2)
        
        if (norm1==0 or norm2==0):
            print('Error in code !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            return 0
        else:
            
            m= (v1[0]*v2[0]) + (v1[1]*v2[1])
            m= math.acos(m/(norm1 * norm2))
            
            return m
            
    def checkCollision(self, vel, i):
        rel_v= [vel[0]- self.obs_vx[i], vel[1]- self.obs_vy[i]]
        rel_s= [self.obs_x[i]-self.currentpose[0], self.obs_y[i]-self.currentpose[1]]
        angle = self.anglebetweenvectors(rel_v, rel_s)
        return abs(angle) < abs(self.cone_angle[i]) #  true implies, collision happening
    
    def checkworkspace(self, vel):
        for i in range (len(self.obs_vx)):
            if self.checkCollision(vel, i):
                return False
        return True
    
    def Createfreespace(self):
        
        self.CollisionConeList()
        
        self.freespace=[[], []]
        
        for i in range(len(self.workspace[0])):
            if (self.checkworkspace([self.workspace[0][i], self.workspace[1][i]])):
                self.freespace[0].append(self.workspace[0][i])
                self.freespace[1].append(self.workspace[1][i])


    def Optima(self):
        ideal_dir= [self.goal[0]-self.currentpose[0], self.goal[1]-self.currentpose[1]]
        
        self.min_val= 2*math.pi

        if (len(self.freespace[0])==0):
            self.min_vel= [0, 0]


        for i in range(len(self.freespace[0])):
            angle = self.anglebetweenvectors([self.freespace[0][len(self.freespace[0])-i-1],self.freespace[1][len(self.freespace[0])-i-1]], ideal_dir)
            if (angle<self.min_val ):
                self.min_val=angle
                self.min_vel= [self.freespace[0][len(self.freespace[0])-i-1],self.freespace[1][len(self.freespace[0])-i-1]]
        
                
    def PublishV(self):
        print('vel_x:',self.min_vel[0],' vel_y:',self.min_vel[1],'--------------------------')
        v_lin, v_ang= self.velocity_convert(self.currentpose[2], self.min_vel[0], self.min_vel[1])
        vel_msg= Twist()
        vel_msg.linear.x=v_lin
        vel_msg.angular.z= v_ang
        print('Vlin:',v_lin, '  |v_ang',v_ang,'--------------------------------------')
        self.pub_vel.publish(vel_msg)
        self.rate.sleep()
        
    def distance(self, p1, p2):
        dist= math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        return dist
        
    def ObstacleAvoidance(self):
        
        
        while(self.distance([self.currentpose[0], self.currentpose[1]], self.goal) > self.epsilon and self.currentpose[0]<4.8):
            self.generateworkspace()
            self.Createfreespace()
            self.Optima()
            self.PublishV()
            
        self.min_vel=[0,0]
        self.PublishV()
        
if __name__ == '__main__':
	rospy.init_node('Assignment3',disable_signals=True)
	rospy.loginfo('Assignment3')
	A3=CollisionAvoidance()
	A3.ObstacleAvoidance()
	rospy.loginfo('done')
	rospy.signal_shutdown('Shutting down node')

    
	plt.plot(A3.trajectory[0],A3.trajectory[1])
	plt.title('Trajectory')
	plt.xlabel('X_coord')
	plt.ylabel('Y_coord')
	plt.show()
	plt.plot(A3.time_abs,A3.trajectory[0])
	plt.title('Variation of X-Coordinate')
	plt.ylabel('pos_x')
	plt.xlabel('time (s)')
	plt.show()
            

            
            
     
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        