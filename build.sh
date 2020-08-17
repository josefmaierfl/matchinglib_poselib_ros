#!/bin/bash

root_node_dir=`pwd`
root_dir="$(cd ${root_node_dir} && cd .. && pwd)"
node_dir1="${root_node_dir}/ait_common"
node_dir2="${root_node_dir}/matchinglib_poselib_ros"

#Create workspace
cd ${root_dir}
mkdir -p ws/src
cd ws/src
catkin_init_workspace
ln -s ${node_dir1}
ln -s ${node_dir2}

#Build ROS node
cd ..
catkin_make
source devel/setup.bash
cd devel
chmod +x setup.bash
./setup.bash
