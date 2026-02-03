#!/bin/bash

# pas sur que ca marche...
WS_ROOT=~/crazyflie_mapping_demo

source /opt/ros/humble/setup.bash
source $WS_ROOT/ros2_ws/install/setup.bash

export GZ_SIM_RESOURCE_PATH="$WS_ROOT/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"

ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py
