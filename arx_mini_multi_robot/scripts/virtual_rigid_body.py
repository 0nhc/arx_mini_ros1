#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import numpy as np
import math

class VRB:
    def __init__(self):
        #subscribe plans for the virtual_rigid_body
        #self.vrb_global_plan_sub = rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, self.vrb_global_plan_callback)
        self.vrb_local_plan_sub = rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.vrb_local_plan_callback, queue_size = 10)

        #self.robot_1_pose = [0.5,0,0,[0,0,0,1]] #[x,y,z,quaternion], relative to the virtual_rigid_body(VRB)
        #self.robot_1_global_plan_pub = rospy.Publisher("/robot1/global_plan", Path,queue_size = 10)
        self.robot_1_local_plan_pub = rospy.Publisher("/robot1/local_plan", Path,queue_size = 10)
        self.robot_1_poses_pub = rospy.Publisher("/robot1/poses", PoseArray,queue_size = 10)

        #self.robot_2_pose = [-0.5,0,0,[0,0,0,1]] #[x,y,z,quaternion], relative to the virtual_rigid_body(VRB)
        #self.robot_2_global_plan_pub = rospy.Publisher("/robot2/global_plan", Path,queue_size = 10)
        #self.robot_2_local_plan_pub = rospy.Publisher("/robot2/local_plan", Path,queue_size = 10)
    
    def transform_matrix_from_quaternion_and_position(self,ox,oy,oz,ow,x,y,z):
        matrix = np.array([[1-2*oy**2-2*oz**2,2*ox*oy-2*oz*ow,2*ox*oz+2*oy*ow,x],
                            [2*ox*oy+2*oz*ow,1-ox**2-oz**2,2*oy*oz-2*ox*ow,y],
                            [2*ox*oz-2*oy*ow,2*oy*oz+2*ox*ow,1-2*ox**2-2*oy**2,z],
                            [0,0,0,1]])
        return matrix
    def quaternion_from_transform(self,mat):
        qw = math.sqrt(1+mat[0][0]+mat[1][1]+mat[2][2])/2
        qx = (mat[2][1]-mat[1][2])/(4*qw)
        qy = (mat[0][2]-mat[2][0])/(4*qw)
        qz = (mat[1][0]-mat[0][1])/(4*qw)
        return qx,qy,qz,qw
        
    def vrb_local_plan_callback(self,data):
        sub_path_1 = Path()
        sub_path_1.header.frame_id = "map"
        sub_path_1.poses = []
        robot_1_pose_array = PoseArray()
        robot_1_pose_array.header.frame_id = "map"
        for i in range(len(data.poses)):   
            vrb_pose = data.poses[i]
            vrb_pose = vrb_pose.pose
            vrb_tf_mat = self.transform_matrix_from_quaternion_and_position(vrb_pose.orientation.x, vrb_pose.orientation.y, vrb_pose.orientation.z, vrb_pose.orientation.w, vrb_pose.position.x, vrb_pose.position.y, vrb_pose.position.z)
            homo_mat = self.transform_matrix_from_quaternion_and_position(0,0,0,1,0.75,0.75,0)
            h = np.matmul(vrb_tf_mat,homo_mat)
            qx,qy,qz,qw = self.quaternion_from_transform(h)
            sub_path_pose = PoseStamped()
            sub_path_pose.pose.orientation.x = qx
            sub_path_pose.pose.orientation.y = qy
            sub_path_pose.pose.orientation.z = qz
            sub_path_pose.pose.orientation.w = qw
            sub_path_pose.pose.position.x = h[0][3]
            sub_path_pose.pose.position.y = h[1][3]
            sub_path_pose.pose.position.z = h[2][3]
            sub_path_pose.header.frame_id = "map"
            sub_path_1.poses.append(sub_path_pose)
            robot_1_pose = Pose()
            robot_1_pose.orientation.x = qx
            robot_1_pose.orientation.y = qy
            robot_1_pose.orientation.z = qz
            robot_1_pose.orientation.w = qw
            robot_1_pose.position.x = h[0][3]
            robot_1_pose.position.y = h[1][3]
            robot_1_pose.position.z = h[2][3]
            robot_1_pose_array.poses.append(robot_1_pose)
        self.robot_1_local_plan_pub.publish(sub_path_1)
        self.robot_1_poses_pub.publish(robot_1_pose_array)

            
                
if __name__ == "__main__":
    rospy.init_node('virtual_rigid_body',anonymous=True)
    vrb = VRB()
    r = rospy.Rate(100)
    r.sleep()
    while not rospy.is_shutdown():
        r.sleep()
