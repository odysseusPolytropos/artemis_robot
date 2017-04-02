#!/bin/bash

sudo apt-get install build-essential ros-jade-gazebo-ros     ros-jade-eigen-conversions     ros-jade-roslint libgazebo5 libsdformat2 libgazebo5-dev ros-jade-ros-control ros-jade-ros-controllers ros-jade-moveit-ros-visualization ros-jade-moveit ros-jade-gazebo-ros-control ros-jade-control-toolbox ros-jade-object-recognition-ros ros-jade-shape-tools ros-jade-xacro ros-jade-shape-msgs ros-jade-robot-state-publisher ros-jade-joint-state-publisher ros-jade-object-recognition-msgs ros-jade-control-msgs ros-jade-nav-msgs ros-jade-moveit-kinematics

sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen.

# we need version 1.1.6, since this version works with ROS Jade, future versions of artemis_robot 
git clone https://github.com/odysseusPolytropos/universal_robot.git
cd universal_robot
git checkout 1.1.6
cd ..

git clone https://github.com/odysseusPolytropos/robotiq_85_gripper.git
git clone https://github.com/odysseusPolytropos/moveit-pkgs.git
git clone https://github.com/odysseusPolytropos/joint-control-pkgs.git
git clone https://github.com/odysseusPolytropos/jaco-arm-pkgs.git
git clone https://github.com/odysseusPolytropos/general-message-pkgs.git
git clone https://github.com/odysseusPolytropos/gazebo-pkgs.git
git clone https://github.com/odysseusPolytropos/general-message-pkgs.git
git clone https://github.com/odysseusPolytropos/convenience-pkgs.git


