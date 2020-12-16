#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
catkin config --extend /opt/ros/$ROS_DISTRO \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DLOG_TRACKING_TIMESTAMPS=ON
catkin config --merge-devel
cd src
wstool init
wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall
wstool update
cd $CATKIN_WS
catkin build -j 2