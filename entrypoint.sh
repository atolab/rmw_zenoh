#!/bin/bash

source ./install/setup.bash

if [ $1 = "router" ]
then
    ros2 run rmw_zenoh_cpp rmw_zenohd
fi

if [ $1 = "talker" ]
then
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    ros2 run demo_nodes_cpp talker
fi

if [ $1 = "listener" ]
then
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    ros2 run demo_nodes_cpp listener
fi

