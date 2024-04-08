#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

class gazebo_messages_converter:
    def __init__(self):
        self.sub_scan = rospy.Subscriber('gazebo/scan', LaserScan, self.laser_callback)
        self.sub_odom = rospy.Subscriber('gazebo/wheel_odom', Odometry, self.odom_callback)
        self.sub_imu = rospy.Subscriber('gazebo/imu/data', Imu, self.imu_callback)

        self.pub_scan = rospy.Publisher('scan', LaserScan, queue_size=1)
        self.pub_odom = rospy.Publisher('wheel_odom', Odometry, queue_size=1)
        self.pub_imu = rospy.Publisher('imu/data', Imu, queue_size=1)
        
        self.robot_name = format((rospy.get_namespace().strip('/')))

    def laser_callback(self, data):
        if(self.robot_name!=""):
            data.header.frame_id = str(self.robot_name)+"/"+str(data.header.frame_id)
        self.pub_scan.publish(data)
        
    def odom_callback(self, data):
        # for unknown reasons, differential driver in gazebo can resolve namespace
        # so there's no need to convert odom messages' frame_ids from gazebo
        
        #if(self.robot_name!=""):
        #    data.header.frame_id = str(self.robot_name)+"/"+str(data.header.frame_id)
        #    data.child_frame_id = str(self.robot_name)+"/"+str(data.child_frame_id)
        self.pub_odom.publish(data)
        
    def imu_callback(self, data):
        if(self.robot_name!=""):
            data.header.frame_id = str(self.robot_name)+"/"+str(data.header.frame_id)
        self.pub_imu.publish(data)

if __name__ == '__main__':
    rospy.init_node('gazebo_messages_converter',anonymous=True)
    rate = rospy.Rate(20)
    cvt = gazebo_messages_converter()
    while not rospy.is_shutdown():
        #rospy.logerr(str(cvt.robot_name)) #for debug
        rate.sleep()
