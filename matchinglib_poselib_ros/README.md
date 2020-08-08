# Table of contents
1. [Introduction](#introduction)
    1. [Developer](#developer)
    2. [Build Environment](#buildenv)
    4. [Overview](#overview)


#  motion_rectification_ros <a name="introduction"></a>

ROS-node to estimate the rectification matrix between to frames.

## key developer <a name="developer"></a>

_Autonomous Systems - Center for Vision, Automation & Control:_

Susanne.Rechbauer@ait.ac.at,  
Josef.Maier.fl@ait.ac.at,  
Roland.Jung@ait.ac.at

## build environment. <a name="buildenv"></a>

Visual Studio 2015 compiler
GCC >= 4.8.2
Cmake >= 3.7
OpenCV == 3.2

PoseLib rev. 5*** (TODO:url)

## Overview  <a name="overview"></a>

Input: 
* image:
  * frame or
  * frame + pose or
  *   frame1 and frame2 or
  * frame1 + pose1 and frame2 and pose 2
* CameraInfo; 
* dyn-parameters: 
   * check-intrinsics (interval[sec] || every-N-frame[int])
   * all parameters from the PoseLib
   
Output: 
* confidence: mean reprojection error
* difference between input and output pose
* pose between two frames
* rectification matrix
* rectified frame1
* rectified frame2

