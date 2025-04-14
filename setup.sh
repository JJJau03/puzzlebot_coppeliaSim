#!/bin/bash

mkdir -p src
mv puzzlebot coppelia_scenes coppelia_sim_ros2 assets src/
colcon build