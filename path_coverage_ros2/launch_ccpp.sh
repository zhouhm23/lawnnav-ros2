#!/bin/zsh
~/.stop_ros.sh
sleep 0.2
rm -f /home/ubuntu/.ros/rtabmap.db
sleep 0.2
ros2 launch navigation rtabmap_navigation.launch.py # 启动视觉导航
sleep 5
ros2 launch navigation rviz_rtabmap_navigation.launch.py # 在rviz中显示
sleep 2
cd ros2_ws/
ros2 launch path_coverage path_coverage.launch.py # 启动路径覆盖程序
sleep 2

wait