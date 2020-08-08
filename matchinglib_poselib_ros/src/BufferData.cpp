#include <ROSnode/BufferData.hpp>

#include <poselib/pose_helper.h>

BufferData::BufferData(ros::Time stamp, cv_bridge::CvImagePtr img)
{
  R_eye = cv::Mat::eye(3,3,CV_64FC1);
  t_zero = cv::Mat::zeros(3,1,CV_64FC1);

  mType = k_mono;
  mTime = stamp;
  mImg0 = img;
}

BufferData::BufferData(ros::Time stamp, cv_bridge::CvImagePtr img, geometry_msgs::PoseStampedConstPtr pose)
{
  R_eye = cv::Mat::eye(3,3,CV_64FC1);
  t_zero = cv::Mat::zeros(3,1,CV_64FC1);

  mType = k_mono_pose;
  mTime = stamp;
  mImg0 = img;
  mPose0 = pose;
}
BufferData::BufferData(ros::Time stamp, cv_bridge::CvImagePtr imgLt, cv_bridge::CvImagePtr imgRt)
{
  R_eye = cv::Mat::eye(3,3,CV_64FC1);
  t_zero = cv::Mat::zeros(3,1,CV_64FC1);

  mType = k_stereo;
  mTime = stamp;
  mImg0 = imgLt;
  mImg1 = imgRt;
}

BufferData::BufferData(ros::Time stamp, cv_bridge::CvImagePtr imgLt, geometry_msgs::PoseStampedConstPtr poseLt, cv_bridge::CvImagePtr imgRt, geometry_msgs::PoseStampedConstPtr poseRt)
{
  R_eye = cv::Mat::eye(3,3,CV_64FC1);
  t_zero = cv::Mat::zeros(3,1,CV_64FC1);

  mType = k_stereo_pose;
  mTime = stamp;
  mImg0 = imgLt;
  mPose0 = poseLt;
  mImg1 = imgRt;
  mPose1 = poseRt;
}

BufferData::~BufferData()
{

}

void BufferData::GetRI(cv::Mat& R)
{
  R = R_eye;
}

void BufferData::Gett0(cv::Mat& t)
{
  t = t_zero;
}

void BufferData::GetImage(cv::Mat& img)
{
  img = mImg0->image;
}

bool BufferData::GetImage1(cv::Mat& img)
{
  if (mType == k_stereo)
  {
    img = mImg1->image;
    return true;
  }

  return false;
}

ros::Time BufferData::GetRosTime()
{
  return mTime;
}

uint64_t BufferData::GetTimeNS()
{
  return mTime.toNSec();
}

bool BufferData::GetAbsPose(geometry_msgs::PoseStampedConstPtr& pose)
{
  if (mType != k_mono_pose)
  {
    return false;
  }

  pose = mPose0;
  return true;
}

bool BufferData::GetRelPose(cv::Mat& R, cv::Mat& t)
{
  // no two poses available -> return false
  if (mType != k_stereo_pose)
  {
    return false;
  }

  cv::Mat q0 = (cv::Mat_<double>(4, 1) << mPose0.get()->pose.orientation.w, mPose0.get()->pose.orientation.x, mPose0.get()->pose.orientation.y, mPose0.get()->pose.orientation.z);
  cv::Mat q1 = (cv::Mat_<double>(4, 1) << mPose1.get()->pose.orientation.w, mPose1.get()->pose.orientation.x, mPose1.get()->pose.orientation.y, mPose1.get()->pose.orientation.z);

  //Check if quaternion is valid (=normalized)
  if(!poselib::nearZero(cv::norm(q0) - 1.0) || !poselib::nearZero(cv::norm(q1) - 1.0))
      return false;

  cv::Mat R0, R1;
  poselib::quatToMatrix(R0, q0);
  poselib::quatToMatrix(R1, q1);

  cv::Mat t0, t1;
  t0 = (cv::Mat_<double>(3,1) << mPose0.get()->pose.position.x, mPose0.get()->pose.position.y, mPose0.get()->pose.position.z);
  t1 = (cv::Mat_<double>(3,1) << mPose1.get()->pose.position.x, mPose1.get()->pose.position.y, mPose1.get()->pose.position.z);

  // determine if given poses are absolute
  /*cv::Mat compR, compt;
  cv::bitwise_xor(R0, R_eye, compR);
  cv::bitwise_xor(t0, t_zero, compt);
  if(cv::countNonZero(compR) > 0 || cv::countNonZero(compt) > 0)*/
  double rdiff = 0, tdiff = 0;
  poselib::getRTQuality(R0, R_eye, t0, t_zero, &rdiff, &tdiff);
  if(!poselib::nearZero(rdiff) || !poselib::nearZero(tdiff))
  {
    // not equal -> abs pose given
    R = R0.t() * R1;
    t = t1 - t0;
  }
  else
  {
    // relative pose given
    R = R1;
    t = t1;
  }

  return true;
}

bool BufferData::GetRelPose(cv::Mat& R, cv::Mat& t, std::shared_ptr<BufferData> d0)
{
  geometry_msgs::PoseStampedConstPtr pose0;

  if (mType != k_mono_pose || !d0->GetAbsPose(pose0))
  {
    return false;
  }

  cv::Mat q0 = (cv::Mat_<double>(4, 1) << pose0.get()->pose.orientation.w, pose0.get()->pose.orientation.x, pose0.get()->pose.orientation.y, pose0.get()->pose.orientation.z);
  cv::Mat q1 = (cv::Mat_<double>(4, 1) << mPose0.get()->pose.orientation.w, mPose0.get()->pose.orientation.x, mPose0.get()->pose.orientation.y, mPose0.get()->pose.orientation.z);

  ROS_INFO("q0: %f - %f - %f - %f", pose0.get()->pose.orientation.w, pose0.get()->pose.orientation.x, pose0.get()->pose.orientation.y, pose0.get()->pose.orientation.z);
  ROS_INFO("q1: %f - %f - %f - %f", mPose0.get()->pose.orientation.w, mPose0.get()->pose.orientation.x, mPose0.get()->pose.orientation.y, mPose0.get()->pose.orientation.z);

  cv::Mat R0, R1;
  poselib::quatToMatrix(R0, q0);
  poselib::quatToMatrix(R1, q1);

  cv::Mat t0, t1;
  t0 = (cv::Mat_<double>(3,1) << pose0.get()->pose.position.x, pose0.get()->pose.position.y, pose0.get()->pose.position.z);
  t1 = (cv::Mat_<double>(3,1) << mPose0.get()->pose.position.x, mPose0.get()->pose.position.y, mPose0.get()->pose.position.z);

  ROS_INFO("pose0: %f - %f - %f", pose0.get()->pose.position.x, pose0.get()->pose.position.y, pose0.get()->pose.position.z);
  ROS_INFO("pose1: %f - %f - %f", mPose0.get()->pose.position.x, mPose0.get()->pose.position.y, mPose0.get()->pose.position.z);

  // absolute poses given
  R = R0.t() * R1;
  ROS_INFO("R0: %f - %f - %f\n   %f - %f - %f\n   %f - %f - %f",
    R0.at<double>(0,0), R0.at<double>(0,1), R0.at<double>(0,2),
    R0.at<double>(1,0), R0.at<double>(1,1), R0.at<double>(1,2),
    R0.at<double>(2,0), R0.at<double>(2,1), R0.at<double>(2,2)
  );
  ROS_INFO("R1: %f - %f - %f\n   %f - %f - %f\n   %f - %f - %f",
    R1.at<double>(0,0), R1.at<double>(0,1), R1.at<double>(0,2),
    R1.at<double>(1,0), R1.at<double>(1,1), R1.at<double>(1,2),
    R1.at<double>(2,0), R1.at<double>(2,1), R1.at<double>(2,2)
  );
  ROS_INFO("R: %f - %f - %f\n   %f - %f - %f\n   %f - %f - %f",
    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
  );
  //t = R*(R1.t()*t1 - R0.t()*t0);
  t = R0.t()*(t1-t0);
  //t = (cv::Mat_(3,1,CV_64FC1) << t1.at<double>(0)-t0.at<double>(0), t1.at<double>(1)-t0.at<double>(1), t1.at<double>(2)-t0.at<double>(2));
  ROS_INFO("t: %f - %f - %f", t.at<double>(0), t.at<double>(1), t.at<double>(2));

  return true;
}

bool BufferData::GetBaseline(double& m, std::shared_ptr<BufferData> d0)
{
  geometry_msgs::PoseStampedConstPtr pose0;

  if (mType != k_mono_pose || !d0->GetAbsPose(pose0))
  {
    return false;
  }

  cv::Mat t0, t1, t;
  t0 = (cv::Mat_<double>(3,1) << pose0.get()->pose.position.x, pose0.get()->pose.position.y, pose0.get()->pose.position.z);
  t1 = (cv::Mat_<double>(3,1) << mPose0.get()->pose.position.x, mPose0.get()->pose.position.y, mPose0.get()->pose.position.z);

  // absolute poses given
  t = t1 - t0;

  m = cv::norm(t);

  return true;
}
