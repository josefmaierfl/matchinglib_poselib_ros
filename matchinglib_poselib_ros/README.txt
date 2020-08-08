/******************************************************************************
* FILENAME:     README.txt
* PURPOSE:      
* AUTHOR:       Josef Maier
* MAIL:         josefjohann-dot-maier-at-gmail-dot-com
* VERSION:      v1.0.0
*
*  Copyright (C) 2017 Austrian Institute of Technologies GmbH - AIT
*  All rights reserved. See the LICENSE file for details.
******************************************************************************/
------------------------------------------------------------------------
ROS-Node template:
------------------------------------------------------------------------
this template node has some features:
# default launch file + default configuration
# dynamic reconfigure
 --> derives from ROSdynamic<template_node::NodeConfig>
 --> it automatically calls update_() if a property changed
 --> properties can easily changed in template_node/cfg/dyn_config.cfg
# derives from the ait_common::Node 
 --> initialization phase (init_topics(), load_config() [from default_config.yaml], init())
 --> cyclic process() [when flag mHasNewMessage was set in a callback]
 --> verbosity control
 --> spinning rate
 --> displays subscripted and published topics
 --> loaded config can be changed in template_node/launch/default_config.yaml
# message sync. example included.

------------------------------------------------------------------------
TO USE THE template_node the name must be replaced on serveral places:
------------------------------------------------------------------------
copy the package and rename the folder/package name <template_node>

template_node/cfg/dyn_config.cfg:
2: PACKAGE = "template_node" 

template_node/include/ROSnode/ROSnode.hpp:
27: #include <template_node/NodeConfig.h>
28: typedef template_node::NodeConfig Config;

template_node/launch/default.launch:
3: <arg name="node_name"       default="template_node" />

template_node/CMakeLists.txt:
3: set(lib_name "template_node")

template_node/package.xml:
3:  <name>template_node</name>

remove the CATKIN_IGNORE file from the package

------------------------------------------------------------------------
     DONT HESITATE TO CONTACT THE AUTHOR FOR FURTHER INFORMATIONS       
------------------------------------------------------------------------
