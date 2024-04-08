#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import tf

rospy.init_node('robot', anonymous=True)

target = rospy.get_param('~target', ('rb1 1.0 0.0')).split(" ")     # 相对目标位置
other_robot = rospy.get_param('~otherRobot', None)                  # 多机防撞，TODO
self_name = rospy.get_param('~selfname', 'rb2')                     # 自身命名空间

obs_range = float(rospy.get_param('~obsRange', 1.0))        # 障碍物感知范围
obs_range_last = obs_range
safe_range = float(rospy.get_param('~safeRange', 0.22))     # 安全距离，离障碍物过近舍弃路径
predict_time = float(rospy.get_param('~predictTime', 0.8))  # 预测时间
dt = float(rospy.get_param('~dt', 0.08))    # 路径段时间间隔
dv = float(rospy.get_param('~dv', 0.15))    # 速度增量
dw = float(rospy.get_param('~dw', 0.9))     # 角速度增量

dh = float(rospy.get_param('~dh', 0.56))    # 末端角度考虑距离
do = float(rospy.get_param('~do', 0.72))    # 障碍物考虑范围

# 动态窗口范围
vrange = rospy.get_param('~v', ('0.3 1.2')).split(' ')  # 速度范围
vrange = list(map(lambda x: float(x), vrange))
wrange = rospy.get_param('~w', ('-4.5 4.5')).split(' ') # 角速度范围
wrange = list(map(lambda x: float(x), wrange))

# DWA评价指标及因数
dist_factor = float(rospy.get_param('~distFactor', 1.0))        # 障碍物距离权重
heading_factor = float(rospy.get_param('~headingFactor', 4.0))  # 朝向目标权重
heading_factor_last = heading_factor
velocity_factor = float(rospy.get_param('~velocityFactor', 1.0))    # 速度权重
w_factor_k1 = float(rospy.get_param('~wFactorK1', 0.1))     # 角速度振荡权重
w_factor_k2 = float(rospy.get_param('~wFactorK2', 0.1))     # 角速度大小权重
target_factor = float(rospy.get_param('~targetFactor', 2.0))    # 距目标点距离权重
target_factor_last = target_factor
switch_dist = float(rospy.get_param('~switchDist', 1.5))    # 切换距离
robot_dist = float(rospy.get_param('~robotDist', 0.7))      # 小车间安全距离

lidar_range = rospy.get_param('~lidarRange', ('-90 90')).split(" ") # 激光考虑范围
lidar_range = list(map(lambda x: float(x), lidar_range))

# 找不到路径时，恢复模式
recover_v = 0
recover_w = -1

# 保存上一时刻角速度
w_t1 = 0
w_t2 = 0

# 增量PID
integralErr = 0
lastErr = 0

def PID(err, kp=2.5, ki=0.01, kd=0.1):
    global integralErr, lastErr
    integralErr += err
    integralErr = min(integralErr, 10)
    result = kp * err + ki * integralErr + kd * (err - lastErr)
    lastErr = err
    return result

# 障碍物类
class Obstacle:
    def __init__(self, v, angle):
        self.num = 1    # 雷达点数
        self.min_angle = angle      # 最小角度
        self.min_angle_dist = v     # 最小角度对应的距离
        self.max_angle = angle      # 最大角度
        self.max_angle_dist = v     # 最大角度对应的距离
        self.min_dist = v           # 最小距离
        self.min_dist_angle = angle # 最小距离对应的角度

    # 增加雷达点，更新障碍物
    def add(self, v, angle):
        self.num += 1
        if v < self.min_dist:
            self.min_dist = v
            self.min_dist_angle = angle
        self.max_angle = angle
        self.max_angle_dist = v

    # 极坐标转笛卡尔坐标系
    def _trans(self, r, t):
        x = r*np.cos(t*np.pi/180)
        y = r*np.sin(t*np.pi/180)
        return (x, y)

    # 计算 点p 到 线段ab 的最短距离
    def _dist(self, a, b, p):
        ab = (b[0]-a[0], b[1]-a[1])
        ap = (p[0]-a[0], p[1]-a[1])
        bp = (p[0]-b[0], p[1]-b[1])
        r = (ap[0]*ab[0] + ap[1]*ab[1]) / (ab[0]**2 + ab[1]**2)
        if r <= 0:
            return pow(ap[0]**2 + ap[1]**2, 0.5)
        elif r > 1:
            return pow(bp[0]**2 + bp[1]**2, 0.5)
        else:
            ac2 = (ab[0]**2 + ab[1]**2) * r**2
            return pow(ap[0]**2 + ap[1]**2 - ac2, 0.5)

    # 获取 点p 到该障碍物的最短距离，用三角形简单估计障碍物
    def minDist(self, p):
        dist1 = self._dist(self._trans(self.min_dist, self.min_dist_angle), self._trans(
            self.max_angle_dist, self.max_angle), p)
        dist2 = self._dist(self._trans(self.min_dist, self.min_dist_angle), self._trans(
            self.min_angle_dist, self.min_angle), p)
        return min(dist1, dist2)

class Robot:
    def __init__(self):
        self.sub_scan = rospy.Subscriber(
            '{}/scan'.format(self_name), LaserScan, self.laser_callback)
        self.sub_pose = rospy.Subscriber(
            '{}/odom'.format(self_name), Odometry, self.pose_callback)
        self.sub_pose1 = rospy.Subscriber(
            '{}/odom'.format(target[0]), Odometry, self.pose_callback1)
        # self.sub_pose2 = rospy.Subscriber(
        #     '{}/odom'.format(other_robot), Odometry, self.pose_callback2)

        self.listener = tf.TransformListener()
        # rospy.wait_for_service('spawn')
        
        self.pub_vel = rospy.Publisher(
            '{}/cmd_vel'.format(self_name), Twist, queue_size=1)

        self.obstacle = []  # 障碍物列表
        # self.pose = {}  # 多机器人位置信息
        self.vel = {}   # 多机器人速度信息

    def laser_callback(self, data):
        last_angle = np.rad2deg(data.angle_min)
        last_v = data.ranges[0]
        self.obstacle = []
        obs_flag = 0
        for i, v in enumerate(data.ranges):
            if v < 0.01 or v > obs_range:   # 滤除不需要的点
                continue
            angle = np.rad2deg(data.angle_increment * i + data.angle_min)
            if angle < lidar_range[0] or angle > lidar_range[1]:    # 滤除角度范围外的点
                continue
            if abs(last_v - v) < 0.1 and abs(last_angle - angle) < 5:   # 若相邻两点距离、角度较近，认为是同一障碍物
                if obs_flag == 1:
                    self.obstacle[-1].add(v, angle) # 障碍物新增点
                else:
                    obs_flag = 1
                    o = Obstacle(v, angle)          # 新增障碍物
                    self.obstacle.append(o)
                    if len(self.obstacle) > 1:
                        if self.obstacle[-2].num < 4:   # 点太少，不考虑该障碍物
                            del self.obstacle[-2]
            else:
                obs_flag = 0    # 连续点断开
            
            # 记录上一个激光雷达点信息
            last_angle = angle
            last_v = v

    # # 坐标转换
    # def pose_transform(self, data):
    #     _, _, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
    #                                        data.pose.pose.orientation.y,
    #                                        data.pose.pose.orientation.z,
    #                                        data.pose.pose.orientation.w])
    #     return np.matrix([
    #         [np.cos(yaw), -np.sin(yaw), 0, data.pose.pose.position.x],
    #         [np.sin(yaw),  np.cos(yaw), 0, data.pose.pose.position.y],
    #         [0,            0,           1, data.pose.pose.position.z],
    #         [0,            0,           0, 1],
    #     ])

    def pose_callback(self, data):
        # self.pose[self_name] = self.pose_transform(data)
        self.vel[self_name] = data.twist.twist

    def pose_callback1(self, data):
        # self.pose[target[0]] = self.pose_transform(data)
        self.vel[target[0]] = data.twist.twist
        
    # def pose_callback2(self, data):
    #     self.pose[other_robot] = self.pose_transform(data)
    #     self.vel[other_robot] = data.twist.twist

    # 局部规划
    def localplan(self):
        try:
            (T, R) = self.listener.lookupTransform('{}/base_link'.format(self_name), '{}/base_link'.format(target[0]), rospy.Time(0))
        except:
            return 0, 0
        # T = self.pose[self_name].I * self.pose[target[0]]   # 两机相对位置
        
        x = T[0]
        y = T[1]
        eular1 = euler_from_quaternion(R)
        T = np.matrix([
            [np.cos(eular1[2]), -np.sin(eular1[2]), 0, T[0]],
            [np.sin(eular1[2]),  np.cos(eular1[2]), 0, T[1]],
            [                0,                  0, 1, T[2]],
            [                0,                  0, 0, 1],
        ])
        # 与目标点相对位置
        T = T * np.matrix([
            [1, 0, 0, -float(target[1])],
            [0, 1, 0, -float(target[2])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        delta_x = float(T[0:1, -1])
        delta_y = float(T[1:2, -1])
        dist = pow(delta_x**2 + delta_y**2, 0.5)    # 离目标点距离
        theta = math.atan2(delta_y, delta_x)        # 离目标点角度
        
        # 在switch_dist以外，倾向于朝目标点前进，且有更大障碍物感知范围
        global heading_factor, target_factor, obs_range
        if dist > switch_dist:
            heading_factor = max(heading_factor_last, target_factor_last)
            target_factor = min(heading_factor_last, target_factor_last)
            obs_range = obs_range_last * 2
        elif dist < switch_dist:
            heading_factor = min(heading_factor_last, target_factor_last)
            target_factor = max(heading_factor_last, target_factor_last)
            obs_range = obs_range_last

        # 停车
        if abs(self.vel[target[0]].linear.x) < 0.05 and abs(self.vel[target[0]].angular.z) < 0.8:
            if dist < 0.3:
                v = 0
                w = 0
                return v, w
            elif pow(x**2 + y**2, 0.5) < robot_dist:
                v = 0
                w = 0
                return v, w
            # elif delta_x < -0.2 and len(self.obstacle) == 0:
            #     v = -0.2
            #     w = 0
            #     return v, w
        
        # 无障碍，纯跟随
        if len(self.obstacle) == 0:
            v = PID(dist)
            w = 4.0 * theta
            return min(v, 2.0), w

        # 计算动态窗口
        vmin = max(vrange[0], self.vel[self_name].linear.x -
                   dv * predict_time // dt)
        wmin = max(wrange[0], self.vel[self_name].angular.z -
                   dw * predict_time // dt)
        vmax = min(vrange[1], self.vel[self_name].linear.x +
                   dv * predict_time // dt)
        wmax = min(wrange[1], self.vel[self_name].angular.z +
                   dw * predict_time // dt)
        
        traj_cache = [0.00001, 0.00001, 0.00001, 0.00001]   # 分数记录，用作评分平滑处理
        valid_traj = []     # 有效路径记录
        max_score = 0       # 最高分数记录
        v = vmin
        w = wmin
        for i in range(int((vmax-vmin)/dv)):
            v = i*dv + vmin
            for j in range(int((wmax-wmin)/dw)):
                w = j*dw + wmin
                traj = Traj(v, w, self.obstacle, theta, x, y)   # 路径生成
                if traj.is_valid:   # 若路径有效
                    traj_cache[0] += traj.dist
                    traj_cache[1] += traj.heading
                    traj_cache[2] += traj.velocity
                    traj_cache[3] += traj.target
                    valid_traj.append(traj)

        if len(valid_traj) == 0:    # 若没有有效路径
            vv = recover_v
            ww = recover_w
            return vv, ww
        
        for traj in valid_traj:
            score = traj.score(traj_cache)
            if score > max_score:
                max_score = score
                vv = traj.v
                ww = traj.w
        return vv, ww

# 路径类
class Traj:
    def __init__(self, v, w, obstacle, theta, x, y, x2=None, y2=None):
        _time = 0
        p = np.array([0, 0, 0])
        self.v = v
        self.w = w
        self.traj = np.array([0, 0, 0])
        self.is_valid = True
        min_dist = 2.0
        while _time <= predict_time:
            # 用直线段拟合
            # p = np.array([p[0]+v*dt*np.cos(p[2]),
            #               p[1]+v*dt*np.sin(p[2]),
            #               p[2]+w*dt])
            
            # 用圆弧拟合
            if w != 0:
                p = np.array([
                    p[0]-v/w*np.sin(p[2])+v/w*np.sin(p[2]+w*dt),
                    p[1]+v/w*np.cos(p[2])-v/w*np.cos(p[2]+w*dt),
                    p[2]+w*dt
                ])
            else:
                p = np.array([
                    p[0]+v*dt,
                    p[1],
                    p[2],
                ])
            
            if _time * v <= do:
                for obs in obstacle:
                    if obs.num < 4:
                        continue
                    if obs.minDist(p) < safe_range: # 距离障碍物太近，路径无效
                        self.is_valid = False
                        if _time == 0:
                            # recover_w朝向离障碍物远的方向
                            global recover_w
                            if obs.min_dist_angle > 0:
                                recover_w = -abs(recover_w)
                            else:
                                recover_w = abs(recover_w)
                        break
                    elif obs.minDist(p) < min_dist:
                        min_dist = obs.minDist(p)
                if pow((p[0]-x)**2 + (p[1]-y)**2, 0.5) < robot_dist:    # 距离领航者太近，路径无效
                    self.is_valid = False
                    break
                        
            if self.is_valid == False:
                break
            
            # 更新路径
            self.traj = np.vstack((self.traj, p))
            _time += dt

        # 评价函数计算
        if self.is_valid:
            self.dist = min_dist
            self.heading = 2*math.pi - (abs(self.traj[min(int(dh/v/dt), int(predict_time/dt))][-1] - theta))
            # self.heading = 2*math.pi - (abs(self.traj[-1][-1] - theta))
            self.velocity = abs(v) + w_factor_k1 * abs(2*w_t1-w-w_t2) - w_factor_k2 * v/vrange[-1] * abs(w)
            self.target = 1 / pow((x-p[0])**2+(y-p[1])**2, 0.5)

    # 分数计算，平滑化
    def score(self, cache):
        self.dist /= cache[0]
        self.heading /= cache[1]
        self.velocity /= cache[2]
        self.target /= cache[3]
        result = dist_factor * self.dist + heading_factor * self.heading + \
            velocity_factor * self.velocity + target_factor * self.target
        return result

if __name__ == '__main__':
    rospy.sleep(2)
    rb = Robot()
    rate = rospy.Rate(20)
    rospy.sleep(3)
    from time import time
    while not rospy.is_shutdown():
        # time0 = time()
        t = Twist()
        v, w = rb.localplan()
        t.linear.x = v
        t.linear.y = 0
        t.angular.z = w
        
        w_t1 = w
        w_t2 = w_t1
        
        rb.pub_vel.publish(t)
        # print(1/(time()-time0))
        rate.sleep()
