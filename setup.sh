#!/bin/bash

cp ~/catkin_ws/src/robocup-restaurant/restaurant/worlds/restaurant.world ~/tiago_public_ws/src/pal_gazebo_worlds/worlds
cp -R ~/tiago_public_ws/src/tiago_navigation/tiago_maps/configurations/small_office ~/tiago_public_ws/src/tiago_navigation/tiago_maps/configurations/restaurant
