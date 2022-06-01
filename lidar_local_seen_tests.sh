#!/usr/bin/env bash
rosparam set /use_sim_time true



timeout 135s roslaunch osm-localization osm_pf_lidar.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/lidar/dynamic/seen/local/config_gaussian.yaml"

timeout 135s roslaunch osm-localization osm_pf_lidar.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/lidar/dynamic/seen/local/config_maplite.yaml"

timeout 135s roslaunch osm-localization osm_pf_lidar.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/lidar/dynamic/seen/local/config_exp.yaml"

timeout 135s roslaunch osm-localization osm_pf_lidar.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/lidar/dynamic/seen/local/config_quadratic.yaml"
