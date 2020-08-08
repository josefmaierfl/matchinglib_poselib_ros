/**
 * Copyright (C) 2016 by Austrian Institute of Technology
 *
 * @file	BufferData.h
 *
 * @brief	Header file for Buffered Data for the Motion Rectification Node
 *
 * @author  Susanne Rechbauer (Susanne.Rechbauer@ait.ac.at)
 *
 * @date  27.07.2017
 */
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
