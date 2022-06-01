#!/usr/bin/env bash
rosparam set /use_sim_time true

timeout 140s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/dynamic/seen/local/config_gaussian.yaml"

timeout 140s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/dynamic/seen/local/config_maplite.yaml"

timeout 140s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/dynamic/seen/local/config_exp.yaml"

timeout 140s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/dynamic/seen/local/config_quadratic.yaml"

# timeout 150s roslaunch osm-localization osm_pf_stereo.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/stereo/dynamic/seen/local/config_gaussian.yaml"



# timeout 150s roslaunch osm-localization osm_pf_stereo.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/stereo/dynamic/seen/local/config_maplite.yaml"

# timeout 150s roslaunch osm-localization osm_pf_stereo.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/stereo/dynamic/seen/local/config_exp.yaml"

# timeout 150s roslaunch osm-localization osm_pf_stereo.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/stereo/dynamic/seen/local/config_quadratic.yaml"
