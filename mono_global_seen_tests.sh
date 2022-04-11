#!/usr/bin/env bash
rosparam set /use_sim_time_true

# timeout 140s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/seen/local/config_gaussian.yaml"

# timeout 140s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/seen/local/config_maplite.yaml"

# timeout 140s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/seen/local/config_exp.yaml"

# timeout 140s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/seen/local/config_quadratic.yaml"

timeout 400s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/seen/global/config_gaussian.yaml"

timeout 400s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/seen/global/config_maplite.yaml"

timeout 400s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/seen/global/config_exp.yaml"

timeout 400s roslaunch osm-localization osm_pf_mono.launch config_file:="/home/mkz/catkin_ws/src/osm-localization/config/mono/seen/global/config_quadratic.yaml"
