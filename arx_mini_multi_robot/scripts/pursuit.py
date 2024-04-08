#!/usr/bin/env python3
# Created on 2022/05/26

import tf
import rospy
import math
import numpy as np
from numpy import cos, sin
from time import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

rospy.init_node('pursuit', anonymous=True)
self_name = rospy.get_param('~selfname', 'arx2')
target = rospy.get_param('~target', ('arx1 1.0 0.0')).split(" ")
start = time()
cache = 0

class Robot:
    def __init__(self):
        self.sub_pose = rospy.Subscriber(
            '{}/odom'.format(self_name), Odometry, self.pose_callback)
        self.sub_pose1 = rospy.Subscriber(
            '{}/odom'.format(target[0]), Odometry, self.pose_callback1)
        self.pub_vel = rospy.Publisher(
            '{}/cmd_vel'.format(self_name), Twist, queue_size=1)
        
        self.vel = {}
        self.listener = tf.TransformListener()
        
    def pose_callback(self, data):
        self.vel[self_name] = data.twist.twist

    def pose_callback1(self, data):
        self.vel[target[0]] = data.twist.twist
        
    def pursuit(self):
        try:
            (T, R) = self.listener.lookupTransform('/map', self_name+'/base_link', rospy.Time())
            (Td, Rd) = self.listener.lookupTransform('/map', target[0]+'/base_link', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Waiting for tf from base_link to base_link...')
            return 0, 0
        
        global start, cache, dt
        t = time() - start
        if t > 2*np.pi:
            start = time()
        
        x = T[0]
        y = T[1]
        _, _, theta = euler_from_quaternion(R)
        
        xl = Td[0]
        yl = Td[1]
        _, _, thetad = euler_from_quaternion(Rd)
        
        xt = float(target[1])
        yt = float(target[2])
        
        # xd = xl - xt*cos(thetad) - yt*sin(thetad)
        # yd = yl - xt*sin(thetad) + yt*cos(thetad)
        
        xd = xl - xt
        yd = yl - yt
        
        vd = self.vel[target[0]].linear.x
        wd = self.vel[target[0]].angular.z
        v  = self.vel[self_name].linear.x
        w  = self.vel[self_name].angular.z
        
        xe =  (x-xd)*cos(theta) + (y-yd)*sin(theta)
        ye = -(x-xd)*sin(theta) + (y-yd)*cos(theta)
        
        thetae = theta - thetad
        if thetae <= -np.pi:
            thetae += 2*np.pi
        elif thetae >= np.pi:
            thetae -= 2*np.pi
        
        if (abs(vd)+abs(wd)) < 0.01:
            cache = 0
        
        cache += (abs(vd)+abs(wd)) * dt
        
        rou = math.exp(-cache)
        h = 6*math.atan(xe**2+ye**2)*sin(t)
        alpha = rou * h
        _thetae = thetae - alpha
        f1 = (sin(thetae) - sin(alpha)) / _thetae
        
        dh = 6*math.atan(xe**2+ye**2)*cos(t)
        dhxe = 12*xe*sin(t) / (1+(xe**2+ye**2)**2)
        dhye = 12*ye*sin(t) / (1+(xe**2+ye**2)**2)
        dalpha = -(abs(vd)+abs(wd))*alpha + rou*dh + rou*(dhxe*(v-vd*cos(thetae)) + dhye*vd*sin(thetae))
        
        k0 = 1
        k1 = 1
        k2 = 1
        
        vv = -k1*xe + vd*cos(thetae)
        ww = -k2*_thetae + wd - k0*vd*ye*f1 + dalpha
        print('Error: ', (xe**2+ye**2)**0.5)
        global errorList
        errorList.append((xe**2+ye**2)**0.5)
        return vv, ww

if __name__ == '__main__':
    rospy.sleep(4)
    rb = Robot()
    rate = rospy.Rate(20)
    dt = 0.05
    rospy.sleep(2)
    errorList = []
    while not rospy.is_shutdown():
        v, w = rb.pursuit()
        t = Twist()
        t.linear.x = v
        t.linear.y = 0
        t.angular.z = w
        rb.pub_vel.publish(t)
        rate.sleep()
        
