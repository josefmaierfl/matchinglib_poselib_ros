//Released under the MIT License - https://opensource.org/licenses/MIT
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

#ifndef ROSNODE_HPP
#define ROSNODE_HPP

//#include <vector>
//#include <utility>
//#include <string>
//#include <memory>
//#include <iostream>

#include <poselib/pose_estim.h>
#include <poselib/stereo_pose_refinement.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
//#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>

#include <std_msgs/Float32.h>

#include <cv_bridge/cv_bridge.h>

#include <ait_common/Node.hpp>
#include <ROSnode/ROSdynamic.hpp>

#include <memory>
#include <boost/circular_buffer.hpp>

#include <ROSnode/BufferData.hpp>
#include <matchinglib_poselib_ros/NodeConfig.h>
typedef matchinglib_poselib_ros::NodeConfig Config;

class ROSnode : public ait_common::Node, public ROSdynamic<Config>
{
public:

  ROSnode(ros::NodeHandle & _nh);
  ~ROSnode();

private:
  // MEMBERS:
  ros::NodeHandle                    mNh;

  // Buffer Data:
  int                                                 mBufferSize = 100;
  boost::circular_buffer<std::shared_ptr<BufferData>> mBuffer;
  ros::Time                          mTime;

  // Init Settings:
  enum eType { k_mono, k_stereo };
  eType                              mType          = k_mono;
  bool                               mbInputPose    = false;
  bool                               mbSkipPoseEst  = false;
  std::weak_ptr<BufferData>          mPrevStereo; // weak ptr ... if element moved out of buffer, take currently inserted element

  // Image Selection Settings:
  enum eSelection { k_all, k_skip, k_time, k_baseline };
  eSelection                         mSelectionType      = k_all;
  int                                mSelectionSkipCount = 0;
  int                                mSelectionTimeMS    = 0;
  int                                mSelectionBaseline  = 0;

  // Subscriber Data
  cv_bridge::CvImage                 mImgLt, mImgRt;
  geometry_msgs::PoseStampedConstPtr mPoseLt;
  cv::Mat                            mR0, mR1, mt0, mt1;
  bool                               mbHasCameraInfo = false;

  // Camera Intrinsics:
  cv::Mat                            mK0, mK1;
  cv::Mat                            mDist0_5, mDist1_5;
  cv::Mat                            mDist0_8, mDist1_8;
  double                             mPxToCamFact = 0.;

  // Settings:
  double                             mRectAlpha          = 0.2;
  std::vector<std::string>           mAvailDetector      = {"FAST", "MSER", "ORB", "BRISK", "KAZE", "AKAZE", "STAR", "MSD", "SIFT", "SURF"};
  std::vector<std::string>           mAvailExtractor     = {"BRISK", "ORB", "KAZE", "AKAZE", "FREAK", "DAISY", "LATCH", "BGM",
                                                            "BGM_HARD",
                                                            "BGM_BILINEAR",
                                                            "LBGM",
                                                            "BINBOOST_64",
                                                            "BINBOOST_128",
                                                            "BINBOOST_256",
                                                            "VGG_120",
                                                            "VGG_80",
                                                            "VGG_64",
                                                            "VGG_48",
                                                            "RIFF",
                                                            "BOLD", "SIFT", "SURF"};
  std::vector<std::string>           mAvailMatcher       = {"GMBSOF", "CASHASH", "HIRCLUIDX", "HIRKMEANS", "LINEAR", "LSHIDX", "RANDKDTREE", "SWGRAPH", "HNSW", "VPTREE", "MVPTREE", "GHTREE", "LISTCLU", "SATREE", "BRUTEFORCENMS", "ANNOY"};
  std::vector<std::string>           mAvailRobustMethod  = {"USAC", "ARRSAC", "RANSAC", "LMEDS"};

  int                                mChosenDetector     = 0;
  int                                mChosenExtractor    = 4;
  int                                mChosenMatcher      = 0;
  int                                mChosenRobustMethod = 0;

  std::string                        mNmsIndex;
  std::string                        mNmsQuery;

  poselib::ConfigUSAC                mUsacConfig;
  int                                mUsacSel[6];
  double                             mUsacTh = 0.85;
  int                                mUSACInlratFilt = 0;

  bool                               mbNoRatioTest       = false;
  bool                               mbRefineVFC         = false;
  bool                               mbRefineVFC_tmp     = false;
  bool                               mbRefineSOF         = false;
  bool                               mbRefineGMS         = false;
  bool                               mbRefineGMS_tmp     = false;
  bool                               mbDynKeyP           = false;
  bool                               mbAutoAdaptTH       = false;

  int                                mRefineSubPx       = 0;
  int                                mRefineRT          = 0;
  int                                mRefineRTweights   = 2;
  int                                mRefineMethod      = poselib::RefinePostAlg::PR_NO_REFINEMENT;
  bool                               mbRefineRTold      = false;
  bool                               mbkneipInsteadBA   = false;

  double                             mth                = 0.8;

  int                                mVerbosityLevel     = 7;

  int                                mMaxKp              = 2000;

  bool                               mbPublishMatches    = true;
  int                                mNumDrawnMatches    = 50;
  int                                mEstimateHomography = 0;
  int                                mRefineBA           = 0;

  bool                               mbHistEqualization  = false;

  double                             mMaxDist3DPtsZ      = 50.0;//Maximum value for the z-coordinates of 3D points to be included into BA. Moreover, this value influences the decision if a pose is marked as stable during stereo refinement (see mMaxRat3DPtsFar).

  //Paramters for stereo refinement
  bool                               mbStereoRef         = false;
  int                                mEvStepStereoStable = 0;
  bool                               mbUseOnlyStablePose = false;
  bool                               mbUseMostLikelyPose = false;
  int                                mRefineRT_stereo    = 4;
  int                                mRefineRTweights_stereo = 2;
  int                                mRefineMethod_stereo = poselib::RefinePostAlg::PR_NO_REFINEMENT;
  bool                               mbRefineRTold_stereo = false;
  bool                               mbkneipInsteadBA_stereo = false;
  int                                mRefineBA_stereo    = 0;
  double                             mMinStartAggInlRat  = 0.2;//Threshold on the inlier ratio for the first pose estimation. Below this threshold the correspondences will be discarded and not used in the next iteration.
  double                             mRelInlRatThLast    = 0.35;//Relative threshold (th = (1 - relInlRatThLast) * last_inl_rat) on the inlier ratios to decide if a new robust estimation is necessary (for comparison, the inlier ratio of the last image pair "last_inl_rat" is used)
  double                             mRelInlRatThNew     = 0.2;//Relative threshold (th = (1 - relInlRatThNew) * old_inl_rat) on the inlier ratios of the new image pair with the old and new (with robust estimation) Essential matrix. Is only used if a new robust estimation was performed (see relInlRatThLast)
  double                             mMinInlierRatSkip   = 0.38;//Absolute threshold on the inlier ratio for new image pairs if the old essential matrix E differs from the new one (estimated with a robust method). Below this threshold, a fall-back threshold estimated by relMinInlierRatSkip can be used, if the resulting threshold is smaller minInlierRatSkip.
  double                             mRelMinInlierRatSkip = 0.7;//Relative threshold (th =  relMinInlierRatSkip * last_valid_inlier_rat) on the inlier ratio for new image pairs compared to the last valid (estimation of E with multiple image pairs) if the old essential matrix E differs from the new one (estimated with a robust method). Below this threshold the old pose is restored.
  size_t                             mMaxSkipPairs       = 5;//Maximum number of times the new Essential matrix E is discarded and restored by the old one (see minInlierRatSkip). If more E's are discarded, the whole system is reinitialized.
  double                             mMinInlierRatioReInit = 0.6;//If the new pose differs from the old, the whole system is reinitialized if the inlier ratio with the new pose is above this value
  float                              mMinPtsDistance     = 3.f;//Minimum distance between points for insertion into the correspondence pool
  size_t                             mMaxPoolCorrespondences = 30000;//Maximum number of correspondences in the correspondence pool after concatenating correspondences from multiple image pairs
  size_t                             mMinContStablePoses = 3;//Number of consecutive estimated poses that must be stable based on a pose distance ranking
  double                             mAbsThRankingStable = 0.075;//Threshold on the ranking over the last minContStablePoses poses to decide if the pose is stable (actual_ranking +/- absThRankingStable)
  bool                               mbUseRANSAC_fewMatches = false;//If true, RANSAC is used for the robust estimation if the number of provided matches is below 150
  size_t                             mCheckPoolPoseRobust = 3;//If not 0, the pose is robustly (RANSAC, ...) estimated from the pool correspondences after reaching checkPoolPoseRobust times the number of initial inliers. A value of 1 means robust estimation is used instead of refinement. For a value >1, the value is exponetially increased after every robust estimation from the pool.
  double                             mMinNormDistStable  = 0.5;//Minimum normalized distance of a stable pose to the center of gravity of all stored poses
  int                                mRaiseSkipCntIncr   = 0;//If not 0, the value of mMaxSkipPairs is increased after a specific number of stable consecutive poses (defined by mRaiseSkipCntStable) were detected. It is increased to std::ceil(mMaxSkipPairs * (1.0 + (mRaiseSkipCntIncr) * 0.25)). The maximum of mRaiseSkipCntIncr=15.
  int                                mRaiseSkipCntStable = 0;//Number of stable consecutive poses ((mRaiseSkipCntStable) + 1) after which mMaxSkipPairs is increased using mRaiseSkipCntIncr.
  double                             mMaxRat3DPtsFar     = 0.5;//Maximum ratio of 3D points for which their z-value is very large (mMaxDist3DPtsZ x baseline) compared to the number of all 3D points. Above this threshold, a pose cannot be marked as stable using only a threshold on the Sampson error ranges (see mAbsThRankingStable)
  bool                               mbNewSettings       = false;

  //Object for stereo refinement and related variables
  std::unique_ptr<poselib::StereoRefine> stereoObj;
  poselib::ConfigPoseEstimation      cfg_stereo;
  int                                evStepStereoStable_tmp = 1;
  int                                evStepStereoStable_cnt = 1;
  bool                               poseWasStable = false;
  bool                               newStereoRefEnable  = false;
  cv::Mat                            R_stable, t_stable;

  // Synchronizers:
  const unsigned                                          mSyncQueueLen = 5;
  typedef message_filters::sync_policies::
          ExactTime<sensor_msgs::Image, geometry_msgs::PoseStamped> syncMonoPose_t;
  typedef message_filters::sync_policies::
          ExactTime<sensor_msgs::Image, sensor_msgs::Image>         syncStereo_t;
  typedef message_filters::sync_policies::
          ExactTime<sensor_msgs::Image, geometry_msgs::PoseStamped,
                    sensor_msgs::Image, geometry_msgs::PoseStamped> syncStereoPose_t;
  typedef message_filters::sync_policies::
          ExactTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> syncCamInfo_t;
  message_filters::Synchronizer<syncMonoPose_t>                     mSyncMonoPose;
  message_filters::Synchronizer<syncStereo_t>                       mSyncStereo;
  message_filters::Synchronizer<syncStereoPose_t>                   mSyncStereoPose;
  message_filters::Synchronizer<syncCamInfo_t>                      mSyncCamInfo;

  // Subscribers:
  ros::Subscriber                                                   mSubImgMono;
  ros::Subscriber                                                   mSubCamInfoMono;
  message_filters::Subscriber<sensor_msgs::Image>                   mSubImgLt;
  message_filters::Subscriber<sensor_msgs::Image>                   mSubImgRt;
  message_filters::Subscriber<geometry_msgs::PoseStamped>           mSubPoseLt;
  message_filters::Subscriber<geometry_msgs::PoseStamped>           mSubPoseRt;
  message_filters::Subscriber<sensor_msgs::CameraInfo>              mSubCamInfoLt;
  message_filters::Subscriber<sensor_msgs::CameraInfo>              mSubCamInfoRt;

  // Publishers:
  ros::Publisher                                                    mPubImgLt;
  ros::Publisher                                                    mPubImgRt;
  ros::Publisher                                                    mPubCamLt;
  ros::Publisher                                                    mPubCamRt;
  ros::Publisher                                                    mPubPose;
  ros::Publisher                                                    mPubPoseDiff;
  ros::Publisher                                                    mPubConfidence;

  // Publisher for Debug:
  ros::Publisher                                                    mPubMatches;

  // callback functions
  void callback_mono            (const sensor_msgs::ImageConstPtr &_img);
  void callback_mono_pose       (const sensor_msgs::ImageConstPtr &_img, const geometry_msgs::PoseStampedConstPtr &_pose);
  void callback_stereo          (const sensor_msgs::ImageConstPtr &_imgLt, const sensor_msgs::ImageConstPtr &_imgRt);
  void callback_stereo_pose     (const sensor_msgs::ImageConstPtr &_imgLt, const geometry_msgs::PoseStampedConstPtr &_poseLt, const sensor_msgs::ImageConstPtr &_imgRt, const geometry_msgs::PoseStampedConstPtr &_poseRt);
  void callback_cam_info_mono   (const sensor_msgs::CameraInfoConstPtr &_cam);
  void callback_cam_info_stereo (const sensor_msgs::CameraInfoConstPtr &_camLt, const sensor_msgs::CameraInfoConstPtr &_camRt);

  // publish functions
  void publish_rectified_images (cv::Mat& imgLt, cv::Mat& imgRt);
  void publish_matches (cv::Mat& img);
  void publish_confidence(double& conf);
  void publish_pose (cv::Mat& R, cv::Mat& t);
  void publish_cam_info (cv::Mat& K0, cv::Mat& d0, cv::Mat& rect0, cv::Mat& K1, cv::Mat& d1, cv::Mat& rect1);

  // helpers
  void draw_matches (cv::Mat& img1, cv::Mat& img2, std::vector<cv::KeyPoint> kp1, std::vector<cv::KeyPoint> kp2, std::vector<cv::DMatch> matches, cv::Mat& outImg);
  void set_usac_config(int id, int selection);
  void configureUSAC(cv::Size imgSize, std::vector<cv::KeyPoint> *kp0, std::vector<cv::KeyPoint> *kp1, std::vector<cv::DMatch> *finalMatches);
  void set_refineRT_config(int id, int selection, bool &refineRTold_, bool &kneipInsteadBA_, int &refineMethod_);
  void set_Stereo_Refine_Options();
  bool select_images_mono(cv::Mat& img0, cv::Mat& R0, cv::Mat& t0, cv::Mat& img1, cv::Mat& R1, cv::Mat& t1);
  bool select_images_stereo(cv::Mat& img0, cv::Mat& R0, cv::Mat& t0, cv::Mat& img1, cv::Mat& R1, cv::Mat& t1);
  void _param_get_sizet(size_t &variable, const std::string &param, const size_t &default_value);

  // DERIVED METHODS:
  bool update_() override final;
  void init_topics();
  void load_config();
  void init_dynamic_config();
  void init();
  void process();
};  // ROSnode


#endif // ROSNODE_HPP
