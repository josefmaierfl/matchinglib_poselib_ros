/**
 * Copyright (C) 2015 by Austrian Institute of Technology
 *
 * @file	obstacle_fusion_node.cpp
 *
 * @brief
 *
 * @author	Roland.Jung.fl@ait.ac.at
 *
 * @date	19.10.2016
 */

#include "ROSnode/ROSnode.hpp"

typedef ROSnode nodeType;

/**
 * The main function of the
 *
 * @param[in] argc argument count
 * @param[in] argv char* to argument list
 *
 */
int main(int argc, char **argv)
{
  std::string name(argv[0]);
  name = name.substr(name.find_last_of('/')+1);
  ROS_INFO("%s started", name.c_str());
  ros::init(argc, argv, name.c_str());
  ros::NodeHandle nh("~");

  nodeType nodeType(nh);
  nodeType.run();

  ROS_INFO("terminated");
}
