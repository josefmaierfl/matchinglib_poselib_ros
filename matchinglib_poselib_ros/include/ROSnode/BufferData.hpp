///Released under the MIT License - https://opensource.org/licenses/MIT
//
//Copyright (c) 2019 AIT Austrian Institute of Technology GmbH
//
//Permission is hereby granted, free of charge, to any person obtaining
//a copy of this software and associated documentation files (the "Software"),
//to deal in the Software without restriction, including without limitation
//the rights to use, copy, modify, merge, publish, distribute, sublicense,
//and/or sell copies of the Software, and to permit persons to whom the
//Software is furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included
//in all copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
//DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
//OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
//USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//Author: Josef Maier (josefjohann-dot-maier-at-gmail-dot-at)

#ifndef BUFFER_DATA_H_
#define BUFFER_DATA_H_

#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>

class BufferData
{
public:

  BufferData(ros::Time stamp, cv_bridge::CvImagePtr img);
  BufferData(ros::Time stamp, cv_bridge::CvImagePtr img, geometry_msgs::PoseStampedConstPtr pose);
  BufferData(ros::Time stamp, cv_bridge::CvImagePtr imgLt, cv_bridge::CvImagePtr imgRt);
  BufferData(ros::Time stamp, cv_bridge::CvImagePtr imgLt, geometry_msgs::PoseStampedConstPtr poseLt, cv_bridge::CvImagePtr imgRt, geometry_msgs::PoseStampedConstPtr poseRt);
  ~BufferData();

  bool GetRelPose(cv::Mat& R, cv::Mat& t);
  bool GetRelPose(cv::Mat& R, cv::Mat& t, std::shared_ptr<BufferData> d0);
  bool GetBaseline(double& m, std::shared_ptr<BufferData> d0);

  ros::Time GetRosTime();
  uint64_t GetTimeNS();
  bool GetAbsPose(geometry_msgs::PoseStampedConstPtr& pose);
  void GetImage(cv::Mat& img);
  bool GetImage1(cv::Mat& img);

  void GetRI(cv::Mat& R);
  void Gett0(cv::Mat& t);

private:
  cv::Mat               R_eye;
  cv::Mat               t_zero;

  enum eType { k_mono , k_mono_pose , k_stereo , k_stereo_pose };
  eType                              mType;
  ros::Time                          mTime;
  cv_bridge::CvImagePtr              mImg0;
  cv_bridge::CvImagePtr              mImg1;
  geometry_msgs::PoseStampedConstPtr mPose0;
  geometry_msgs::PoseStampedConstPtr mPose1;
}; // BufferData

#endif // BUFFER_DATA_H_
