#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory 
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist

class cooperative_grasp:
    def __init__(self):
        self.pub_arx1_joint1 = rospy.Publisher('/arx1/joint1_position_controller/command', JointTrajectory, queue_size=1)
        self.pub_arx1_joint2 = rospy.Publisher('/arx1/joint2_position_controller/command', JointTrajectory, queue_size=1)
        self.pub_arx1_joint3 = rospy.Publisher('/arx1/joint3_position_controller/command', JointTrajectory, queue_size=1)
        self.pub_arx1_joint4 = rospy.Publisher('/arx1/joint4_position_controller/command', JointTrajectory, queue_size=1)
        self.pub_arx1_gripper = rospy.Publisher('/arx1/gripper_position_controller/command', JointTrajectory, queue_size=1)
        self.pub_arx1_vel = rospy.Publisher('/arx1/cmd_vel', Twist, queue_size=1)
        
        self.pub_arx2_joint1 = rospy.Publisher('/arx2/joint1_position_controller/command', JointTrajectory, queue_size=1)
        self.pub_arx2_joint2 = rospy.Publisher('/arx2/joint2_position_controller/command', JointTrajectory, queue_size=1)
        self.pub_arx2_joint3 = rospy.Publisher('/arx2/joint3_position_controller/command', JointTrajectory, queue_size=1)
        self.pub_arx2_joint4 = rospy.Publisher('/arx2/joint4_position_controller/command', JointTrajectory, queue_size=1)
        self.pub_arx2_gripper = rospy.Publisher('/arx2/gripper_position_controller/command', JointTrajectory, queue_size=1)
        self.pub_arx2_vel = rospy.Publisher('/arx2/cmd_vel', Twist, queue_size=1)
        
        self.robot_name = format((rospy.get_namespace().strip('/')))

if __name__ == '__main__':
    rospy.init_node('gazebo_messages_converter',anonymous=True)
    rate = rospy.Rate(10)
    yi_wu = cooperative_grasp()
    rospy.loginfo(rospy.get_time())
    rospy.loginfo(rospy.get_time())
    start_time=rospy.get_time()

    while not rospy.is_shutdown():
        
        time_delta=rospy.get_time()-start_time
        rospy.loginfo(time_delta)

        if time_delta>13:
            break
        # arx1_joint1
        traj = JointTrajectory()
        trajp = JointTrajectoryPoint()
        traj.joint_names.append("joint1")
        trajp.positions = [3.141593]
        trajp.velocities = [0]
        trajp.accelerations= [0]
        trajp.effort= [0]
        trajp.time_from_start.secs = 10
        trajp.time_from_start.nsecs = 0
        traj.points.append(trajp)
        yi_wu.pub_arx1_joint1.publish(traj)
        
        # arx1_joint2
        traj = JointTrajectory()
        trajp = JointTrajectoryPoint()
        traj.joint_names.append("joint2")
        trajp.positions = [-0.785]
        trajp.velocities = [0]
        trajp.accelerations= [0]
        trajp.effort= [0]
        trajp.time_from_start.secs = 10
        trajp.time_from_start.nsecs = 0
        traj.points.append(trajp)
        yi_wu.pub_arx1_joint2.publish(traj)
        
        # arx1_joint3
        traj = JointTrajectory()
        trajp = JointTrajectoryPoint()
        traj.joint_names.append("joint3")
        trajp.positions = [0.785]
        trajp.velocities = [0]
        trajp.accelerations= [0]
        trajp.effort= [0]
        trajp.time_from_start.secs = 10
        trajp.time_from_start.nsecs = 0
        traj.points.append(trajp)
        yi_wu.pub_arx1_joint3.publish(traj)
   
        
        # arx2_joint2
        traj = JointTrajectory()
        trajp = JointTrajectoryPoint()
        traj.joint_names.append("joint2")
        trajp.positions = [-0.785]
        trajp.velocities = [0]
        trajp.accelerations= [0]
        trajp.effort= [0]
        trajp.time_from_start.secs = 10
        trajp.time_from_start.nsecs = 0
        traj.points.append(trajp)
        yi_wu.pub_arx2_joint2.publish(traj)
        
        # arx2_joint3
        traj = JointTrajectory()
        trajp = JointTrajectoryPoint()
        traj.joint_names.append("joint3")
        trajp.positions = [0.785]
        trajp.velocities = [0]
        trajp.accelerations= [0]
        trajp.effort= [0]
        trajp.time_from_start.secs = 10
        trajp.time_from_start.nsecs = 0
        traj.points.append(trajp)
        yi_wu.pub_arx2_joint3.publish(traj)
                # arx1_gripper
        traj = JointTrajectory()
        trajp = JointTrajectoryPoint()
        traj.joint_names.append("gripper")
        trajp.positions = [0.02]
        trajp.velocities = [0.01]
        trajp.accelerations= [0]
        trajp.effort= [0]
        trajp.time_from_start.secs = 1
        trajp.time_from_start.nsecs = 0
        traj.points.append(trajp)
        yi_wu.pub_arx1_gripper.publish(traj)
                # arx1_gripper
        traj = JointTrajectory()
        trajp = JointTrajectoryPoint()

        traj.joint_names.append("gripper")
        trajp.positions = [0.02]
        trajp.velocities = [0.01]
        trajp.accelerations= [0]
        trajp.effort= [0]
        trajp.time_from_start.secs = 1
        trajp.time_from_start.nsecs = 0
        traj.points.append(trajp)
        yi_wu.pub_arx2_gripper.publish(traj)
        

        
        rate.sleep()

    while not rospy.is_shutdown():
        time_delta=rospy.get_time()-start_time
        if time_delta>15:
            break
        # arx1_gripper
        traj = JointTrajectory()
        trajp = JointTrajectoryPoint()
        traj.joint_names.append("gripper")
        trajp.positions = [-0.01]
        trajp.velocities = [0.01]
        trajp.accelerations= [0]
        trajp.effort= [0]
        trajp.time_from_start.secs = 1
        trajp.time_from_start.nsecs = 0
        traj.points.append(trajp)
        yi_wu.pub_arx1_gripper.publish(traj)
        
        
        # arx2_gripper
        traj = JointTrajectory()
        trajp = JointTrajectoryPoint()
        traj.joint_names.append("gripper")
        trajp.positions = [-0.01]
        trajp.velocities = [0.01]
        trajp.accelerations= [0]
        trajp.effort= [0]
        trajp.time_from_start.secs = 1
        trajp.time_from_start.nsecs = 0
        traj.points.append(trajp)
        yi_wu.pub_arx2_gripper.publish(traj)
        
        rate.sleep()
