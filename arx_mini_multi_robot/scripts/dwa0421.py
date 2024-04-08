#!/usr/bin/env python3

import tf
import rospy
import math
import numpy as np
from time import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

ONLY_DWA = True
ONLY_FOLLOW = False

rospy.init_node('robot', anonymous=True)

target = rospy.get_param('~target', ('arx1 0.7 0.0')).split(" ")    # 相对目标位置
L0 = pow(float(target[1])**2 + float(target[2])**2, 0.5)
other_robot = rospy.get_param('~otherRobot', None)                  # 多机防撞，TODO
self_name = rospy.get_param('~selfname', 'arx2')                    # 自身命名空间

# 跟随参数
k_v = 2.0   # 距离误差比例系数
k_w = 3.0   # 角度误差比例系数

# DWA基本参数
obs_range = float(rospy.get_param('~obsRange', 1.8))        # 障碍物感知范围
min_lidar_point = float(rospy.get_param('~minPoint', 8))
safe_range = float(rospy.get_param('~safeRange', 0.22))     # 安全距离，离障碍物过近舍弃路径
predict_time = float(rospy.get_param('~predictTime', 0.9))  # 预测时间
predict_time_nav = float(rospy.get_param('~predictTimeNav', 0.3))   # 预测领航者时间
obs_range_last = obs_range
predict_time_last = predict_time

dt = float(rospy.get_param('~dt', 0.1))    # 路径段时间间隔
dv = float(rospy.get_param('~dv', 0.14))    # 速度增量
dw = float(rospy.get_param('~dw', 0.25))     # 角速度增量

dh = float(rospy.get_param('~dh', 0.6))    # 末端角度考虑距离
do = float(rospy.get_param('~do', 0.8))    # 障碍物考虑范围
dh_last = dh

# 动态窗口范围
vrange = rospy.get_param('~v', ('0.01 1.0')).split(' ')  # 速度范围
vrange = list(map(lambda x: float(x), vrange))
wrange = rospy.get_param('~w', ('-2.5 2.5')).split(' ') # 角速度范围
wrange = list(map(lambda x: float(x), wrange))

# DWA评价指标及因数
dist_factor = float(rospy.get_param('~distFactor', 1.25))        # 障碍物距离权重
heading_factor = float(rospy.get_param('~headingFactor', 2.0))  # 朝向领航者权重
velocity_factor = float(rospy.get_param('~velocityFactor', 1.0))    # 速度权重
w_factor_k1 = float(rospy.get_param('~wFactorK1', 0.1))             # 角速度振荡权重
w_factor_k2 = float(rospy.get_param('~wFactorK2', 0.1))             # 角速度大小权重
target_factor = float(rospy.get_param('~targetFactor', 5.0))    # 距领航者距离权重
same_factor= float(rospy.get_param('~sameFactor', 0.1))         # 相同速度权重
switch_dist = float(rospy.get_param('~switchDist', 1.4))    # 切换距离
robot_dist = float(rospy.get_param('~robotDist', 0.3))      # 小车间安全距离
dist_factor_last = dist_factor
heading_factor_last = heading_factor
target_factor_last = target_factor
velocity_factor_last = velocity_factor
same_factor_last = same_factor

lidar_range = rospy.get_param('~lidarRange', ('-80 80')).split(" ") # 激光考虑范围
lidar_range = list(map(lambda x: float(x), lidar_range))

# 找不到路径时，恢复模式
recover_v = 0
recover_w = 1
recover_flag = 0
recover_start = time()
recover_time = (0.5, 1.0)   # 自转时间

# 保存上一时刻角速度
w_t1 = 0
w_t2 = 0

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
        self.pub_path = rospy.Publisher(
            '{}/dwaPath'.format(self_name), Path, queue_size=1)
        self.pub_target = rospy.Publisher(
            '{}/targetLink'.format(self_name), Path, queue_size=1)

        self.obstacle = []  # 障碍物列表
        # self.pose = {}  # 多机器人位置信息
        self.vel = {}   # 多机器人速度信息
        
        while(1):
            try:
                t, r = self.listener.lookupTransform(self_name+'/base_link', self_name+'/laser', rospy.Time())
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo('Waiting for tf from base_link to laser...')
        _, _, yaw = euler_from_quaternion(r)
        self.laser_yaw = yaw
        print(yaw)

    def laser_callback(self, data):
        # if time() - data.header.stamp.secs > 1:
        #     rospy.logwarn('Laser Data Too Old')
        last_angle = np.rad2deg(data.angle_min + self.laser_yaw)
        last_v = data.ranges[0]
        self.obstacle = []
        obs_flag = 0
        for i, v in enumerate(data.ranges):
            angle = np.rad2deg(data.angle_increment * i + data.angle_min + self.laser_yaw)
            if v < 0.01 or v > obs_range:   # 滤除不需要的点
                continue
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
                        if self.obstacle[-2].num < min_lidar_point:   # 点太少，不考虑该障碍物
                            del self.obstacle[-2]
            else:
                obs_flag = 0    # 连续点断开
            
            # 记录上一个激光雷达点信息
            last_angle = angle
            last_v = v
        
        if len(self.obstacle) > 0:
            if self.obstacle[-1].num < min_lidar_point:
                del self.obstacle[-1]

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
            (T, R) = self.listener.lookupTransform(self_name+'/base_link', target[0]+'/base_link', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('Waiting for tf from base_link to base_link...')
            return 0, 0
        
        x = T[0]
        y = T[1]
        
        self.show_target((x, y))
        
        dist = pow(x**2+y**2, 0.5)

        theta = math.atan2(y, x)        # 离领航者角度
        
        global heading_factor, target_factor, obs_range, predict_time, dh, dist_factor, velocity_factor, same_factor
        if dist > switch_dist + 0.2:
            heading_factor = heading_factor_last
            target_factor = target_factor_last
            dist_factor = dist_factor_last
            velocity_factor = velocity_factor_last
            same_factor = 0
            predict_time = predict_time_last * 1.2
            obs_range = obs_range_last * 1.2
            dh = dh_last
        elif dist < switch_dist - 0.2:
            target_factor = target_factor_last / 3
            heading_factor = heading_factor_last * 3
            velocity_factor = velocity_factor_last / 3
            same_factor = same_factor_last
            dh = dh_last / 2
            dist_factor = dist_factor_last / 3
            obs_range = obs_range_last
            
        # 停车
        if abs(self.vel[target[0]].linear.x) < 0.05 and abs(self.vel[target[0]].angular.z) < 0.8:
            if dist < robot_dist:
                v = 0
                w = theta if abs(theta) > 0.1 else 0
                rospy.loginfo('Stop')
                return v, w
            if abs(dist - L0) < 0.3:
                v = 0
                w = theta if abs(theta) > 0.1 else 0
                rospy.loginfo('Stop')
                return v, w
        
        # 纯跟随
        if not ONLY_DWA:
        #     try:
        #         (trans, rot) = self.listener.lookupTransform(target[0]+'/base_link', self_name+'/base_link', rospy.Time())
        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #         rospy.logwarn('Waiting for tf from base_link to base_link...')
        #         return 0, 0
        #     delta_x = trans[0]
        #     delta_y = trans[1]
        #     delta_theta = euler_from_quaternion(rot)[2]
        #     theta0 = math.atan2(-float(target[2]), -float(target[1]))
        #     err_x = L0 * math.cos(theta0) - delta_x
        #     err_y = L0 * math.sin(theta0) - delta_y
        #     w_leader = self.vel[target[0]].angular.z
        #     v_leader = self.vel[target[0]].linear.x
        #     v_follower = (k_1 * err_x - delta_y * w_leader + v_leader) * math.cos(delta_theta) + (k_2 * err_y + delta_x * w_leader) * math.sin(delta_theta)
        #     w_follower = ((k_2 * err_y + delta_x * w_leader) * math.cos(delta_theta) - (k_1 * err_x - delta_y * w_leader + v_leader) * math.sin(delta_theta))/follow_d

            v_follower = k_v * (dist - L0)
            w_follower = k_w * theta

        if ONLY_FOLLOW:
            rospy.loginfo('Follow Mode')
            return v_follower, w_follower
        
        # 无障碍，纯跟随
        if len(self.obstacle) == 0 and not ONLY_DWA:
            rospy.loginfo('Follow Mode')
            return v_follower, w_follower
            
        # 计算动态窗口
        vmin = max(vrange[0], self.vel[self_name].linear.x -
                   dv * predict_time // dt)
        wmin = max(wrange[0], self.vel[self_name].angular.z -
                   dw * predict_time // dt)
        vmax = min(vrange[1], self.vel[self_name].linear.x +
                   dv * predict_time // dt)
        wmax = min(wrange[1], self.vel[self_name].angular.z +
                   dw * predict_time // dt)
        
        nav_v = self.vel[target[0]].linear.x
        if nav_v+0.2 > vmin and dist < 2.0:
            vmax = min(vmax, nav_v+0.32)
        
        traj_cache = [0.00001, 0.00001, 0.00001, 0.00001, 0.00001]   # 分数记录，用作评分平滑处理
        valid_traj = []     # 有效路径记录
        max_score = 0       # 最高分数记录
        v = vmin
        w = wmin
        
        # 预测领航者
        _time = 0
        nav_traj = np.array([0, 0, 0])
        p = np.array([0, 0, 0])
        nav_v = self.vel[target[0]].linear.x
        nav_w = self.vel[target[0]].angular.z
        while _time <= predict_time_nav:
            # 用圆弧拟合
            if nav_w != 0:
                p = np.array([
                    p[0]-nav_v/nav_w*np.sin(p[2])+nav_v/nav_w*np.sin(p[2]+nav_w*dt),
                    p[1]+nav_v/nav_w*np.cos(p[2])-nav_v/nav_w*np.cos(p[2]+nav_w*dt),
                    p[2]+nav_w*dt
                ])
            else:
                p = np.array([
                    p[0]+nav_v*dt,
                    p[1],
                    p[2],
                ])
            nav_traj = np.vstack((nav_traj, p))
            _time += dt
        
        P_b = nav_traj[-1]
        P_b[-1] = 1
        P_b = P_b.reshape(3, 1)
        _, _, yaw = euler_from_quaternion(R)
        T_ba = np.matrix([
            [np.cos(yaw), -np.sin(yaw), x],
            [np.sin(yaw),  np.cos(yaw), y],
            [          0,            0, 1],
        ])
        P_a = T_ba * P_b
        nav_point = (float(P_a[0]), float(P_a[1]))
        
        for i in range(int((vmax-vmin)/dv)):
            v = i*dv + vmin
            for j in range(int((wmax-wmin)/dw)):
                w = j*dw + wmin
                traj = Traj(v, w, self.obstacle, theta, x, y, nav_point, nav_v, nav_w)   # 路径生成
                if traj.is_valid:   # 若路径有效
                    traj_cache[0] += traj.dist
                    traj_cache[1] += traj.heading
                    traj_cache[2] += traj.velocity
                    traj_cache[3] += traj.target
                    valid_traj.append(traj)

        if len(valid_traj) == 0:    # 若没有有效路径
            vv = recover_v
            ww = recover_w
            global recover_flag, recover_start, recover_time
            recover_flag = 1
            if time() - recover_start < recover_time[-1] + 0.2:
                recover_flag = 2
            recover_start = time()
            rospy.loginfo('Recover Mode')
            return vv, ww
        
        for traj in valid_traj:
            score = traj.score(traj_cache)
            if score > max_score:
                max_score = score
                vv = traj.v
                ww = traj.w
        rospy.loginfo('DWA Mode | Obstacle Num: {}'.format(len(self.obstacle)))
        self.show_path(traj)
        return vv, ww
        
    def show_path(self, traj):
        path = Path()
        path.header.frame_id = self_name+'/base_link'
        for t in traj.traj:
            p = PoseStamped()
            p.header.frame_id = self_name+'/base_link'
            p.pose.position.x = t[0]
            p.pose.position.y = t[1]
            path.poses.append(p)
        self.pub_path.publish(path)
        
    def show_target(self, target):
        path = Path()
        path.header.frame_id = self_name+'/base_link'
        p1 = PoseStamped()
        p1.header.frame_id = self_name+'/base_link'
        p1.pose.position.x = 0
        p1.pose.position.y = 0
        path.poses.append(p1)
        p2 = PoseStamped()
        p2.header.frame_id = self_name+'/base_link'
        p2.pose.position.x = target[0]
        p2.pose.position.y = target[1]
        path.poses.append(p2)
        self.pub_target.publish(path)

# 路径类
class Traj:
    def __init__(self, v, w, obstacle, theta, x, y, nav_point, nav_v, nav_w, x2=None, y2=None):
        _time = 0
        p = np.array([0, 0, 0])
        self.v = v
        self.w = w
        self.traj = np.array([0, 0, 0])
        self.is_valid = True
        min_dist = 2.0
        while _time <= predict_time:
            
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
                    if obs.num < min_lidar_point:
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
            self.target = 1 / (abs(pow((nav_point[0]-p[0])**2+(nav_point[1]-p[1])**2, 0.5) - L0 + 0.05) + 0.01)
            self.same = 1 / (abs(v - nav_v) + 0.01)

    # 分数计算，平滑化
    def score(self, cache):
        self.dist /= cache[0]
        self.heading /= cache[1]
        self.velocity /= cache[2]
        self.target /= cache[3]
        self.same /= cache[4]
        result = dist_factor * self.dist + heading_factor * self.heading + \
                velocity_factor * self.velocity + target_factor * self.target + \
                same_factor * self.same
        return result

if __name__ == '__main__':
    rospy.sleep(1)
    rb = Robot()
    rate = rospy.Rate(10)
    rospy.sleep(2)
    v_t1 = 0
    while not rospy.is_shutdown():
        t = Twist()
        
        if recover_flag:
            v, w = v_t1, w_t1
            if time() - recover_start >= recover_time[recover_flag-1]:
                recover_flag = 0
        else:
            v, w = rb.localplan()
        t.linear.x = v
        t.linear.y = 0
        t.angular.z = w
        
        w_t1 = w
        v_t1 = v
        w_t2 = w_t1
        rb.pub_vel.publish(t)
        rate.sleep()
