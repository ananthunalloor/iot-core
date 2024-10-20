#!/bin/bash

echo "building sub_demo"

cd /home/ros2_ws/sub_demo
colcon build
source /home/ros2_ws/sub_demo/install/setup.bash

echo "sub_demo built complete"

# ros2 run sub_demo subscriber_demo