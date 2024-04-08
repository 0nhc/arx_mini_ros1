#!/bin/sh


gnome-terminal -- bash -c " roslaunch arx_mini_bringup bringup.launch  robot_name:=arx1;"

gnome-terminal -- bash -c "sleep 3;roslaunch arx_mini_navigation localization.launch robot_name:=arx1"
