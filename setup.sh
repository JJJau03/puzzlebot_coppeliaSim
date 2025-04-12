#!/bin/bash

mkdir -p src
mv puzzlebot coppelia_scenes coppelia_sim_ros2 src/
colcon build