#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

target_pos = np.array([0.8, 0, 0, 0])
integralErr = 0
lastErr = 0
max_vel = 3.0

class Follower:
    def __init__(self):
        rospy.init_node('follower')
        self.pub_vel = rospy.Publisher('arx2/cmd_vel', Twist, queue_size=1)
        self.sub_arx1_vel = rospy.Subscriber('arx1/cmd_vel', Twist, self.arx1_vel_callback)
        self.sub_arx1_pose = rospy.Subscriber('arx1/odom', Odometry, self.arx1_pose_callback)
        self.sub_arx2_pose = rospy.Subscriber('arx2/odom', Odometry, self.arx2_pose_callback)
        self.arx1_vel = Twist()
        
    def arx1_pose_callback(self, data):
        _, _, yaw = euler_from_quaternion([data.pose.pose.orientation.x, 
                                           data.pose.pose.orientation.y,
                                           data.pose.pose.orientation.z,
                                           data.pose.pose.orientation.w])
        pose = data.pose.pose.position
        self.pose1 = np.matrix([
            [np.cos(yaw), -np.sin(yaw), 0, pose.x],
            [np.sin(yaw),  np.cos(yaw), 0, pose.y],
            [          0,            0, 1, pose.z],
            [          0,            0, 0,      1],
        ])
        
    def arx2_pose_callback(self, data):
        _, _, yaw = euler_from_quaternion([data.pose.pose.orientation.x, 
                                           data.pose.pose.orientation.y,
                                           data.pose.pose.orientation.z,
                                           data.pose.pose.orientation.w])
        pose = data.pose.pose.position
        self.pose2 = np.matrix([
            [np.cos(yaw), -np.sin(yaw), 0, pose.x],
            [np.sin(yaw),  np.cos(yaw), 0, pose.y],
            [          0,            0, 1, pose.z],
            [          0,            0, 0,      1],
        ])
        
    def arx1_vel_callback(self, data):
        self.arx1_vel = data
        
def PID(err, kp=8.0, ki=0.005, kd=8.0):
    global integralErr, lastErr
    integralErr += err
    integralErr = min(integralErr, 100)
    result = kp * err + ki * integralErr + kd * (err - lastErr)
    lastErr = err
    return result

if __name__ == '__main__':
    arx2 = Follower()
    rate = rospy.Rate(20)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        
        T = arx2.pose2.I * arx2.pose1
        delta_x = float(T[0:1, -1]) - target_pos[0]
        delta_y = float(T[1:2, -1]) - target_pos[1]
        
        # # PID
        t = Twist()
        t.linear.x = PID(delta_x)
        t.angular.z = math.atan2(delta_y, delta_x)
        
        if abs(arx2.arx1_vel.linear.x) < 0.02 and abs(arx2.arx1_vel.angular.z) < 0.5 and math.sqrt(delta_x**2+delta_y**2) < 0.05:
            t.linear.x = 0
            t.angular.z = 0
            integralErr = 0
            
        t.linear.x = max(min(t.linear.x, max_vel), -max_vel)
        arx2.pub_vel.publish(t)
        print(delta_x, delta_y)
        
        rate.sleep()
