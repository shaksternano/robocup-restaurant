#!/bin/bash

cd ~/tiago_public_ws
source devel/setup.bash
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true world:=restaurant
