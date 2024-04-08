#!/bin/sh

gnome-terminal -- bash -c "source /opt/ros/noetic/setup.bash;
                           source ~/arx_ws/devel/setup.bash;
                           roslaunch arx_mini_navigation navigation.launch robot_name:=arx2; exec bash"

gnome-terminal -- bash -c "sleep 2;source /opt/ros/noetic/setup.bash;
                           source ~/arx_ws/devel/setup.bash;
                           rosparam load '/home/arx2/arx_ws/src/arx_mini_ros1/arx_mini_bringup/param/bridge.yaml' ;
                           roslaunch arx_mini_bringup arx_teleop.launch robot_name:=arx2; exec bash"

gnome-terminal -- bash -c "sleep 5;source /opt/ros/foxy/setup.bash;
                           source /opt/ros/foxy/setup.bash;
                           source ~/colcon_ws/install/setup.bash;
                           ros2 run ros1_bridge parameter_bridge ;exec bash"
