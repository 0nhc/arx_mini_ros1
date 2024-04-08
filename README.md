# ARX_MINI_ROS

This is a ROS1 package for ARX_MINI. <br>

## 0. Modifications for rcbigcar

I have modified some parts in **[rcbigcar](arx_mini_base/rcbigcar)** to make it work for multi-robot developments.<br>

* In **[rcbigcar/src.robot.motor.cpp](arx_mini_base/rcbigcar/src/robot/motor.cpp)**, line106-113, I filtered and published the error detected by encoders because there is a static bias (value 0.3141592741012573) which leads to self-rotation when there is no moving command. The topic name is **/motorerror**.

```c++
//Error Filter
if(error!=0)
error = error - 0.3141592741012573;

//Publish Error
std_msgs::Float32 data;
data.data = error;
error_pub.publish(data);
```



* The frame ids of **odom** topic and **imu** topic in previous code is static and can't be changed. Now I changed it as ROS param input. Remember to set the frame ids in launch file. I have already given an example in **[arx_mini_bringup/launch/bringup.launch](arx_mini_bringup/launch/bringup.launch)**

```xml
<node name="robot" pkg="rcbigcar" type="robot" respawn="true" clear_params="true">
    <param name="IsDebug" 		value="$(arg is_debug)" />
    <param name="imu_frame_id" 		value="$(arg frame_prefix)imu" />
    <param name="odom_header_frame_id" 	value="$(arg frame_prefix)odom" />
    <param name="odom_child_frame_id" 	value="$(arg frame_prefix)base_link" />
</node>
```



## 1. Make Sure CAN BUS is Configured Properly

```sh
source $(directory to arx_mini_ros1)/arx_mini_base/rcbigcar/scripts/setup_can.sh
```



## 2. Only Launch the Robot Base

You can decide the robot's name space when you launch this file.<br>All the topics, nodes and TF frames will be added with this name space.<br>

### 2.1 In Real Word

```sh
# for example the robot is called arx1
roslaunch arx_mini_bringup bringup.launch robot_name:=arx1
# for example the robot is called arx2
roslaunch arx_mini_bringup bringup.launch robot_name:=arx2
...
```

Then you can control with your keyboard<br>

```sh
# for example you want to control arx1
roslaunch arx_mini_multi_robot teleop.launch robot_name:=arx1
# for example you want to control arx2
roslaunch arx_mini_multi_robot teleop.launch robot_name:=arx2
...
```

### 2.2 In Simulation

Generate one robot to play<br>

```sh
roslaunch arx_mini_gazebo gazebo.launch
```

You can easily generate multiple robots following the launch file format like **[two_robots_gazebo.launch](arx_mini_gazbeo/launch/two_robots_gazebo.launch)**<br>

```xml
<?xml version="1.0"?>
<launch>
  <!-- ==================== LAUNCH GAZEBO WORLD ==================== -->
  <include file="$(find arx_mini_gazebo)/launch/bringup_world.launch"/>
  
  <!-- ===================== GENERATE ROBOT 1 ===================== -->
  <include file="$(find arx_mini_gazebo)/launch/bringup_robot.launch">
    <arg name="robot_name" 	value="arx1" />
    <arg name="x_pos" 		value="0.0"/>
    <arg name="y_pos" 		value="0.0"/>
    <arg name="z_pos" 		value="0.06"/>
    <arg name="yaw"   		value="0.0"/>
  </include>
  
  <!-- ===================== GENERATE ROBOT 2 ===================== -->
  <include file="$(find arx_mini_gazebo)/launch/bringup_robot.launch">
    <arg name="robot_name" 	value="arx2" />
    <arg name="x_pos" 		value="-0.4"/>
    <arg name="y_pos" 		value="0.0"/>
    <arg name="z_pos" 		value="0.06"/>
    <arg name="yaw"   		value="0.0"/>
  </include>
</launch>
```

Then you can control a certain robot with keyboard<br>

```sh
# for example you want to control arx1
roslaunch arx_mini_multi_robot teleop.launch robot_name:=arx1
# for example you want to control arx2
roslaunch arx_mini_multi_robot teleop.launch robot_name:=arx2
...
```

## 3. Localization with Maps

### 3.1 In Real World

```sh
# for example you want to get the position of arx1
roslaunch arx_mini_bringup bringup.launch robot_name:=arx1
roslaunch arx_mini_navigation localization.launch robot_name:=arx1
# for example you want to get the position of arx2
roslaunch arx_mini_bringup bringup.launch robot_name:=arx2
roslaunch arx_mini_navigation localization.launch robot_name:=arx2
...
```

### 3.2 In Simulation

```sh
# for example you want to get the position of arx1
roslaunch arx_mini_gazebo gazebo.launch robot_name:=arx1
roslaunch arx_mini_navigation localization.launch robot_name:=arx1 use_sim_time:=true
# for example you want to get the position of arx2
roslaunch arx_mini_fgazebo gazebo.launch robot_name:=arx2
roslaunch arx_mini_navigation localization.launch robot_name:=arx2 use_sim_time:=true
...
```

## 4. Navigation of a Single Robot

### 4.1 In Real World

```sh
# Terminal 1
roslaunch arx_mini_bringup bringup.launch
# Terminal 2
roslaunch arx_mini_navigation navigation.launch 
# Terminal 3 (Optional)
roslaunch arx_mini_navigation multi_goals.launch
```

### 4.2 In Simulation

```sh
# Terminal 1
roslaunch arx_mini_gazebo gazebo.launch
# Terminal 2
roslaunch arx_mini_navigation navigation.launch use_sim_time:=true
# Terminal 3 (Optional)
roslaunch arx_mini_navigation multi_goals.launch use_sim_time:=true
```

## 5. Multi Robot Fleet

To test in simulation, just

```sh
# Terminal 1
roslaunch arx_mini_multi_robot two_robots.launch

# Terminal 2 
roslaunch arx_mini_multi_robot teleop.launch robot_name:=arx1
```

## 6. References

### laser_scan_matcher_odometry

[https://github.com/bierschi/laser_scan_matcher_odometry](https://github.com/bierschi/laser_scan_matcher_odometry)<br>

### robot_localization

[https://github.com/cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization)<br>

### iris_lama_ros

[https://github.com/iris-ua/iris_lama_ros](https://github.com/iris-ua/iris_lama_ros)<br>

### yocs_velocity_smoother

[https://github.com/yujinrobot/yujin_ocs](https://github.com/yujinrobot/yujin_ocs)<br>
