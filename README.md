# ROS Interface for MATCHING- AND POSELIB

- [Introduction](#introduction)
- [Installation](#installation)
  - [Dependencies](#dependencies)
  - [Installation on Linux Systems](#system-dependencies)
- [Build ROS node for MATCHING- AND POSELIB](#build)
- [Node topics](#node-topics)
  - [Input](#node-input)
  - [Output](#node-output)
- [Parameters](#parameters)
- [Usage](#usage)
- [Publication](#publication)

## Introduction <a name="introduction"></a>

This repository provides a ROS interface for library [matchinglib_poselib](https://github.com/josefmaierfl/matchinglib_poselib) to create sparse image feature matches, estimate camera poses, and calculate rectified images.

For details on available functionalities see the library description of [MATCHING- AND POSELIB](https://github.com/josefmaierfl/matchinglib_poselib).

The ROS interface can be configured to accept mono or stereo images and, thus, supports matching only (returns images side-by-side with drawn matches in [OpenCV style](https://docs.opencv.org/3.4/d4/d5d/group__features2d__draw.html#ga7421b3941617d7267e3f2311582f49e1)), [Motion Stereo](https://link.springer.com/chapter/10.1007/978-1-4612-2834-9_4), single mono/stereo pose estimations, and continuous high accurate stereo pose estimation and refinement.

Parameters can be adjusted by modifying [./matchinglib_poselib_ros/launch/default_config.yaml](./matchinglib_poselib_ros/launch/default_config.yaml) and/or by dynamic reconfiguration during runtime.

## Installation <a name="installation"></a>

The interface was tested on Ubuntu 12.04 to 18.04.

### Dependencies <a name="dependencies"></a>

This ROS interface depends on the following libraries/frameworks:
* [ROS](https://www.ros.org)
* [MATCHING- AND POSELIB](https://github.com/josefmaierfl/matchinglib_poselib)
* [OpenCV 4.2.0](https://opencv.org/)
* [Eigen 3.3.7](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [SBA](https://github.com/balintfodor/sba)
* [CLAPACK](https://github.com/NIRALUser/CLAPACK)

All mentioned dependencies except ROS are also dependencies to [MATCHING- AND POSELIB](https://github.com/josefmaierfl/matchinglib_poselib).

### Install dependencies on Linux Systems <a name="system-dependencies"></a>

1. Please follow instructions on [installing dependencies for MATCHING- AND POSELIB](https://github.com/josefmaierfl/matchinglib_poselib#system-dependencies)
2. [Install MATCHING- AND POSELIB](https://github.com/josefmaierfl/matchinglib_poselib#library)
3. Install [ROS](https://www.ros.org/install/)
4. Install xterm: `sudo apt-get install xterm`

## Build ROS node for MATCHING- AND POSELIB <a name="build"></a>

1. Create and/or change to your ROS working directory (e.g. `mkdir ~/work && cd ~/work`).
2. Clone the repository: `git clone https://github.com/josefmaierfl/matchinglib_poselib_ros.git`
3. Change into cloned repository: `cd matchinglib_poselib_ros`
4. Build ROS nodes by executing `./build.sh` or by performing steps within [./build.sh](./build.sh)

## Node topics <a name="node-topics"></a>

Used topics make use of [sensor_msgs](http://docs.ros.org/melodic/api/sensor_msgs/html/index-msg.html) and [geometry_msgs](http://docs.ros.org/melodic/api/geometry_msgs/html/index-msg.html) messages:
* [CameraInfo](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html): Distortion model `plumb_bob`
* [Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html): Encoding `mono8`
* [PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)

### Input <a name="node-input"></a>

#### Stereo Camera Case

* `/cam0/image`: Type `sensor_msgs/Image`
* `/cam1/image`: Type `sensor_msgs/Image`
* `/cam0/camera_info`: Type `sensor_msgs/CameraInfo` (provide camera matrix and distortion coefficients)
* `/cam1/camera_info`: Type `sensor_msgs/CameraInfo` (provide camera matrix and distortion coefficients)

Optional (for comparison to estimated pose):
* `/cam0/pose`: Type `geometry_msgs/PoseStamped`
* `/cam1/pose`: Type `geometry_msgs/PoseStamped`

#### Mono Camera Case

* `/cam0/image`: Type `sensor_msgs/Image`
* `/cam0/camera_info`: Type `sensor_msgs/CameraInfo` (provide camera matrix and distortion coefficients)

Optional (for comparison to estimated pose):
* `/cam0/pose`: Type `geometry_msgs/PoseStamped`

### Output <a name="node-output"></a>

Rectified images:
* `/matchinglib_poselib_ros/cam0/rect`: Type `sensor_msgs/Image`
* `/matchinglib_poselib_ros/cam1/rect`: Type `sensor_msgs/Image`

Camera matrices and distortion coefficients:
* `/matchinglib_poselib_ros/cam0/camera_info`: Type `sensor_msgs/CameraInfo`
* `/matchinglib_poselib_ros/cam1/camera_info`: Type `sensor_msgs/CameraInfo`

Estimated pose:
* `/matchinglib_poselib_ros/pose`: Type `geometry_msgs/PoseStamped`

Optional pose difference (if enabled and reference pose provided):
* `/matchinglib_poselib_ros/pose_diff`: Type `geometry_msgs/PoseStamped`

Optional (if enabled) - feature matches in [OpenCV style](https://docs.opencv.org/3.4/d4/d5d/group__features2d__draw.html#ga7421b3941617d7267e3f2311582f49e1):
* `/matchinglib_poselib_ros/matches`: Type `sensor_msgs/Image`

## Parameters <a name="parameters"></a>

Parameters can be set by editing file [./matchinglib_poselib_ros/launch/default_config.yaml](./matchinglib_poselib_ros/launch/default_config.yaml) before launching `matchinglib_poselib_ros` and/or by dynamic reconfiguration during runtime.

Details/descriptions on each parameter can be found in the launch file [./matchinglib_poselib_ros/launch/default_config.yaml](./matchinglib_poselib_ros/launch/default_config.yaml) or by hovering over parameter names in `rqt_reconfigure` window.

## Usage <a name="usage"></a>

After [adopting parameters](#parameters):
1. Start the ROS core service: `roscore`
2. Start advertising [above listed topics](#node-input) (e.g. using a bag-file and `rosbag play`).
3. Start ROS node `matchinglib_poselib_ros`:
```
cd [your_working_directory]/ws/devel
source setup.bash
./setup.bash
roslaunch matchinglib_poselib_ros default.launch
```
4. Optional: Start `rviz` to show input and output images
5. Optional: Run `rosrun rqt_reconfigure rqt_reconfigure` for dynamic reconfiguration of parameters
6. Optional: Record advertised topics using `rosbag record -O <file-name> /matchinglib_poselib_ros/cam0/rect /matchinglib_poselib_ros/cam1/rect /matchinglib_poselib_ros/cam0/camera_info /matchinglib_poselib_ros/cam1/camera_info /matchinglib_poselib_ros/pose`

## Publication <a name="publication"></a>

Please cite the following papers if you use SemiRealSequence or parts of this code in your own work.

```
@inproceedings{maier2016guided,
  title={Guided Matching Based on Statistical Optical Flow for Fast and Robust Correspondence Analysis},
  author={Maier, Josef and Humenberger, Martin and Murschitz, Markus and Zendel, Oliver and Vincze, Markus},
  booktitle={European Conference on Computer Vision},
  pages={101--117},
  year={2016},
  organization={Springer}
}
```

```
@inproceedings{maier2017ground,
  title={Ground truth accuracy and performance of the matching pipeline},
  author={Maier, Josef and Humenberger, Martin and Zendel, Oliver and Vincze, Markus},
  booktitle={Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition Workshops},
  pages={29--39},
  year={2017}
}
```
