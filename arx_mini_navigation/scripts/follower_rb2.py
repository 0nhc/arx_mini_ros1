#!/usr/bin/env python3
#encoding=utf-8

import rospy
import numpy as np
from time import time
import actionlib
import math
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID

# 目标相对位置x，y
target_pos = np.array([0.6, 0])

# PID误差
integralErr = 0
lastErr = 0

class Follower:
    def __init__(self):
        rospy.init_node('follower2')
        
        # 跟随的参照物，arx1
        self.sub_arx1_pose = rospy.Subscriber('arx1/odom', Odometry, self.arx1_pose_callback)
        
        # 跟随者，arx2
        self.sub_arx2_pose = rospy.Subscriber('arx2/odom', Odometry, self.arx2_pose_callback)
        self.sub_arx2_scan = rospy.Subscriber('arx2/scan', LaserScan, self.arx2_lidar_callback)

        self.pub_vel = rospy.Publisher('arx2/cmd_vel', Twist, queue_size=1)

        self.arx2_lidar = LaserScan()
        self.arx1_vel = Twist()

        # DWA预测时间，增大会增加计算时间
        self.predict_time = 0.6
        
        # DWA时间间隔，减小会增加计算时间
        self.dt = 0.1
        
        self.pub_traj = rospy.Publisher('arx2/localplanner', Path, queue_size=1)
        
    # 给定v, w生成路径
    def traj_generate(self, v, w):
        time = 0
        x = np.array([0, 0, 0])	# 相对于自身，[x方向坐标, y方向坐标, 角度theta]
        traj = np.array([0, 0, 0])	# 路径

        # 使用多段小直线段来模拟路径
        while time <= self.predict_time:
            x = np.array([x[0]+v*self.dt*np.cos(x[2]),	# x	x = x + v*t*cos(theta)
                          x[1]+v*self.dt*np.sin(x[2]), 	# y	y = y + v*t*sin(theta)
                          x[2]+w*self.dt])			# theta theta = theta + w*t
            traj = np.vstack((traj, x))	# 更新路径
            time += self.dt
        return traj
        
    # 订阅自身雷达信息,回调函数
    def arx2_lidar_callback(self, data):
    
        # 是否存在障碍物 标志
        self.obstacle_flag = 0
        flag = 0
        
        global delta_x, delta_y, w0
        
        # 遍历雷达信息，判断周围是否存在障碍物
        for i, v in enumerate(data.ranges):
            # 滤除部分雷达点，减少计算开销
            if i % 5 != 0:
                continue
            
            # 距离判断，感知0.7m范围内激光雷达点
            if v < 0.01 or v > 0.7:
                continue
                
            # 角度判断
            angle = 90 - np.rad2deg(data.angle_increment * i + data.angle_min)
            if abs(90 - angle) > 90:
                continue
                
            # 若还未跳出循环，说明周围存在障碍物，标志置1
            flag = 1
            break
            
        if flag == 0:
            pass
        else:
        # 若存在障碍物，应用dwa避障
            self.obstacle_flag = 1
            
            # 恢复模式，找不到合适路径时的行为
            self.valid_v = 0
            self.valid_w = -1.5
            
            max_score = -1000.0	# 最高分数记录
            osc = 0			# 振荡判断
            
            # 速度、角速度采样
            for v in (0.2, 0.3, 0.6, 0.8, 1.0, 1.2, 1.5):
                for w in (0, -0.2, 0.2, -0.5, 0.5, -0.8, 0.8, -1.2, 1.2, 1.4, -1.4):
                    _min = 3.0	# 最小距离记录
                    flag = 1		# 路径是否距离障碍物过近，标记
                    traj = self.traj_generate(v, w)	# 路径生成
                    
                    # 这里遍历计算障碍物距离,时间复杂度n^2,感觉可以优化
                    for i in range(len(traj)):
                        if flag == 0:	# 距离障碍物太近，舍弃路径
                            break
                        for j, vv in enumerate(data.ranges):
                            # 滤除部分雷达点，减少计算开销
                            if j % 20 != 0:	
                                continue
                            if vv == float('inf') or vv > 1.0 or vv < 0.01:
                                continue
                            angle = 90 - np.rad2deg(data.angle_increment * j + data.angle_min)	# 激光雷达点角度计算
                            if abs(90 - angle) > 80:
                                continue
                            xx = vv * np.sin(angle*np.pi/180)	# 相对于小车的x方向位置
                            yy = vv * np.cos(angle*np.pi/180)	# 相对于小车的y方向位置
                            current_dist = math.sqrt((traj[i,0] - xx)**2 + (traj[i,1] - yy)**2)	# 当前路径点i与激光雷达点的距离计算
                            if current_dist > 0.12:	# 距离大于0.12m则认为路径可行
                                if current_dist < _min:
                                    _min = current_dist	# 保存所有路径点中离障碍物最近的距离
                                continue
                            flag = 0
                            break
                        if flag == 0:	# 距离障碍物太近，舍弃路径
                            break
                    if flag == 0:	# 距离障碍物太近，舍弃路径
                        continue
                    if w0 * w < 0:	# 若角速度发生突变，给予一定惩罚
                        osc = 0.001
                        
                    # 分数计算：与障碍物的距离 & 路径末端与目标相对位置的距离 & 速度 & 振荡惩罚；分数越高，路径越优
                    score = _min - 0.18 * pow((traj[-1,0] - delta_x)**2 + (traj[-1,1] - delta_y)**2, 0.5) + 0.01 * v - osc
                    if max_score < score:
                        max_score = score
                        self.valid_v = v
                        self.valid_w = w
                        
            # 路径显示
            path = Path()
            path.header.frame_id = 'arx2/base_link'
            path.poses = []
            for p in self.traj_generate(self.valid_v, self.valid_w):
                pose = PoseStamped()
                pose.header.frame_id = 'arx2/base_link'
                pose.pose.position.x = p[0]
                pose.pose.position.y = p[1]
                pose.pose.position.z = 0
                path.poses.append(pose)
            self.pub_traj.publish(path)
                        
        
    def arx1_pose_callback(self, data):
        _, _, yaw = euler_from_quaternion([data.pose.pose.orientation.x, 
                                           data.pose.pose.orientation.y,
                                           data.pose.pose.orientation.z,
                                           data.pose.pose.orientation.w])
        pose = data.pose.pose.position
        
        # arx1齐次变换矩阵，相对于map
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
        
        # arx2齐次变换矩阵，相对于map
        self.pose2 = np.matrix([
            [np.cos(yaw), -np.sin(yaw), 0, pose.x],
            [np.sin(yaw),  np.cos(yaw), 0, pose.y],
            [          0,            0, 1, pose.z],
            [          0,            0, 0,      1],
        ])
        
# 位置式pid
def PID(err, kp=2.1, ki=0.001, kd=0.01):
    global integralErr, lastErr
    integralErr += err
    integralErr = min(integralErr, 10)
    result = kp * err + ki * integralErr + kd * (err - lastErr)
    lastErr = err
    return result
    
if __name__ == '__main__':
    delta_x = 0
    delta_y = 0
    arx2 = Follower()
    rate = rospy.Rate(20)
    v0 = 0
    w0 = 0
    rospy.sleep(8)
    while not rospy.is_shutdown():
        time0 = time()
        
        T = arx2.pose2.I * arx2.pose1	# T：arx1相对于arx2的齐次变换矩阵
        
        delta_x = float(T[0:1, -1]) - target_pos[0]	# 跟随者坐标系下目标点x坐标
        delta_y = float(T[1:2, -1]) - target_pos[1]	# 跟随者坐标系下目标点y坐标
        beta = math.atan2(delta_y, delta_x)		# atan2(y, x)，跟随者坐标系下目标点的相对角度
        dist = pow(delta_x**2 + delta_y**2, 0.5)	# 相对目标点距离

	# 跟随
        v = PID(dist)
        w = 2.0 * beta
        
        # 若存在障碍物，dwa避障；将 == 1 改为 == 其他数字 可以禁用避障，测试跟随效果
        if arx2.obstacle_flag == 1:
            v = arx2.valid_v
            w = arx2.valid_w
            
        # 领航者停止，跟随者停止
        if abs(arx2.arx1_vel.linear.x) < 0.05 and abs(arx2.arx1_vel.angular.z) < 0.8:
            if dist < 0.2:
                v = 0
                w = 0
            elif delta_x < -0.2:
                v = -0.2

	# 速度发布
        t = Twist()
        t.linear.x = min(v, 1.8)
        t.angular.z = w
        arx2.pub_vel.publish(t)
        
        # 记录上一次的v, w
        v0 = min(v, 1.8)
        w0 = w
        
        rate.sleep()
        
        
   
        
