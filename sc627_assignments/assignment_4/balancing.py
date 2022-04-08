#!/usr/bin/env python

import rospy
import actionlib
from math import sqrt,atan2,cos
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import time,sleep
import matplotlib.pyplot as plt


class balancing:
    
    def __init__(self):
        #For the current robot
        self.pose_bot_i= [0, 0, 0]
        self.v_bot_i= 0
        self.w_bot_i= 0
        #variables for robot to the left
        self.pose_left= [0, 0, 0]
        self.v_left= 0
        self.w_left= 0
        #variables for robot to the right
        self.pose_right= [0, 0, 0]
        self.v_right= 0
        self.w_right= 0
        
        self.rate=rospy.Rate(50)
        
        self.trajectory= [[], []]
        self.time_abs= []
        self.start = 0
        
        self.v_max= 0.15
        self.w_max= 0.8
        self.error= 0.0005
        
        rospy.Subscriber('/odom', Odometry, self.callback_boti) 
        rospy.Subscriber('/left_odom', Odometry, self.callback_leftbot)
        rospy.Subscriber('/right_odom', Odometry, self.callback_rightbot) 
        
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        
        
    def callback_boti(self,msg):
        self.pose_bot_i[0]= msg.pose.pose.position.x
        self.pose_bot_i[1]= msg.pose.pose.position.y
        self.v_bot_i= msg.twist.twist.linear.x
        self.w_bot_i= msg.twist.twist.angular.z
        
        self.time_abs.append(time()-self.start)
        self.trajectory[0].append(self.pose_bot_i[0])
        self.trajectory[1].append(self.pose_bot_i[1])
        
    def callback_leftbot(self, msg):
        self.pose_left[0]= msg.pose.pose.position.x
        self.pose_left[1]=msg.pose.pose.position.y
        self.v_left= msg.twist.twist.linear.x
        self.w_left= msg.twist.twist.angular.z
        
    def callback_rightbot(self, msg):
        self.pose_right[0]= msg.pose.pose.position.x
        self.pose_right[1]=msg.pose.pose.position.y
        self.v_right= msg.twist.twist.linear.x
        self.w_right= msg.twist.twist.angular.z
        
    def velocity_convert(self, vel_x, vel_y):
        v_lin= min(max(vel_x, -self.v_max), self.v_max)
        v_ang=0
        return v_lin, v_ang
    
    def publishVelocity(self, vel):
        (v_lin, v_ang)=self.velocity_convert(vel[0], vel[1])
        v_msg= Twist()
        v_msg.linear.x=v_lin
        v_msg.angular.z=v_ang
        self.pub_vel.publish(v_msg)
    
    def balancing_algo(self):
        self.start=time()
        M = 0 
        while(self.v_bot_i>self.error and self.v_bot_i>self.error and self.v_bot_i>self.error or M<1000):
            u = (self.pose_right[0]-self.pose_bot_i[0])-(self.pose_bot_i[0]-self.pose_left[0])
            self.publishVelocity([u, 0])
            M= M+1
            self.rate.sleep()
        self.publishVelocity([0,0])


if __name__ == '__main__':
    # Defining ROS node
    rospy.init_node('Bot_i',disable_signals=True)
    rospy.loginfo('Node for bot i')
    Bot_i_control= balancing()
    rospy.loginfo('Bot Control run')
    Bot_i_control.balancing_algo()
    plt.plot(Bot_i_control.trajectory[0],Bot_i_control.trajectory[1])
    plt.title('Trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()
    plt.plot(Bot_i_control.time_abs, Bot_i_control.trajectory[0])
    plt.title('X coordinate wrt time')
    plt.xlabel('time')
    plt.ylabel('X_coord')
    plt.show()
    
   




