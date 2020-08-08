#include <ROSnode/ROSnode.hpp>

#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/next_prior.hpp>

#include <poselib/pose_helper.h>
#include <poselib/pose_homography.h>
#include "poselib/pose_linear_refinement.h"
#include <matchinglib/matchinglib.h>
#include <matchinglib/vfcMatches.h>
#include "matchinglib/gms.h"
#include <matchinglib/matchinglib_correspondences.h>

#define PX_MIN_GOOD_TH 0.8 //If the pixel start threshold chosen is too small to give a result this is checked by this "normal" pixel threshold

ROSnode::ROSnode(ros::NodeHandle & _nh) :
  Node(_nh), mNh(_nh), mSyncMonoPose(mSyncQueueLen), mSyncStereo(mSyncQueueLen), mSyncStereoPose(mSyncQueueLen), mSyncCamInfo(mSyncQueueLen) //, mSyncQueueLen(5)
{
  // init usac config
  //mUsacConfig.th_pixels = PIX_MIN_GOOD_TH;

  // load config from yaml file:
  load_config();

  // init poses
  mR0 = cv::Mat::eye(3,3,CV_64FC1);
  mR1 = cv::Mat::eye(3,3,CV_64FC1);

  mt0 = cv::Mat::zeros(3,1,CV_64FC1);
  mt1 = cv::Mat::zeros(3,1,CV_64FC1);
}

ROSnode::~ROSnode()
{

}

void ROSnode::init_topics()
{


  //char c;
  //std::cout << "press a key..." << std::endl;
  //std::cin >> c;

  // Subscribers:
  if (mType == k_mono)
  {
    mSubCamInfoMono = mNh.subscribe<sensor_msgs::CameraInfo>("/cam0/camera_info", mSyncQueueLen, &ROSnode::callback_cam_info_mono, this);

    if (!mbInputPose)
    {
      mSubImgMono = mNh.subscribe<sensor_msgs::Image>("/cam0/image", mSyncQueueLen, &ROSnode::callback_mono, this);
    }
    else
    {
      mSubImgLt.subscribe(mNh, "/cam0/image", mSyncQueueLen);
      mSubPoseLt.subscribe(mNh, "/cam0/pose", mSyncQueueLen);

      mSyncMonoPose.connectInput(mSubImgLt, mSubPoseLt);
      mSyncMonoPose.registerCallback(boost::bind(&ROSnode::callback_mono_pose, this, _1, _2));
    }
  }
  else
  {
    mSubImgLt.subscribe(mNh, "/cam0/image", mSyncQueueLen);
    mSubImgRt.subscribe(mNh, "/cam1/image", mSyncQueueLen);
    mSubCamInfoLt.subscribe(mNh, "/cam0/camera_info", mSyncQueueLen);
    mSubCamInfoRt.subscribe(mNh, "/cam1/camera_info", mSyncQueueLen);

    mSyncCamInfo.connectInput(mSubCamInfoLt, mSubCamInfoRt);
    mSyncCamInfo.registerCallback(boost::bind(&ROSnode::callback_cam_info_stereo, this, _1, _2));

    if (!mbInputPose)
    {
      mSyncStereo.connectInput(mSubImgLt, mSubImgRt);
      mSyncStereo.registerCallback(boost::bind(&ROSnode::callback_stereo, this, _1, _2));
    }
    else
    {
      mSubPoseLt.subscribe(mNh, "/cam0/pose", mSyncQueueLen);
      mSubPoseRt.subscribe(mNh, "/cam1/pose", mSyncQueueLen);

      mSyncStereoPose.connectInput(mSubImgLt, mSubPoseLt, mSubImgRt, mSubPoseRt);
      mSyncStereoPose.registerCallback(boost::bind(&ROSnode::callback_stereo_pose, this, _1, _2, _3, _4));
    }
  }

  mPubImgLt      = mNh.advertise<sensor_msgs::Image>("cam0/rect", mSyncQueueLen);
  mPubImgRt      = mNh.advertise<sensor_msgs::Image>("cam1/rect", mSyncQueueLen);

  mPubCamLt      = mNh.advertise<sensor_msgs::CameraInfo>("cam0/camera_info", mSyncQueueLen);
  mPubCamRt      = mNh.advertise<sensor_msgs::CameraInfo>("cam1/camera_info", mSyncQueueLen);

  mPubPose       = mNh.advertise<geometry_msgs::PoseStamped>("pose", mSyncQueueLen);

  if (mbInputPose)
  {
    mPubPoseDiff   = mNh.advertise<geometry_msgs::PoseStamped>("pose_diff", mSyncQueueLen);
  }

  mPubConfidence = mNh.advertise<std_msgs::Float32>("confidence", mSyncQueueLen);

  mPubMatches    = mNh.advertise<sensor_msgs::Image>("matches", mSyncQueueLen);
}

void ROSnode::load_config()
{
  std::string text;
  int tmp = 0;

  mNh.param<int>("bufferSize", tmp, 100);
  if (tmp > 0)
  {
    mBufferSize = tmp;
  }
  mBuffer = boost::circular_buffer<std::shared_ptr<BufferData>>(mBufferSize);

  mNh.param<std::string>("type", text, "");
  if (text == "mono")
  {
    mType = k_mono;
  }
  else if (text == "stereo")
  {
    mType = k_stereo;
  }
  else
  {
    mType = k_mono;
    ROS_INFO("LoadConfig: Type not given. Subscribing to single camera image instead.");
  }

  mNh.param<bool>("bInputPose", mbInputPose, false);

  mNh.param<bool>("bSkipPose", mbSkipPoseEst, false);

  if (mbSkipPoseEst && !mbInputPose)
  {
    mbSkipPoseEst = false;
    ROS_WARN("No input pose is given. Pose estimation will be activated.");
  }

  mNh.param<std::string>("selectionType", text, "");
  if (text == "All")
  {
    mSelectionType = k_all;
  }
  else if (text == "Skip")
  {
    mSelectionType = k_skip;
  }
  else if (text == "Time")
  {
    mSelectionType = k_time;
  }
  else if (text == "Baseline")
  {
    mSelectionType = k_baseline;
  }

  mNh.param<int>("skipCount", mSelectionSkipCount, 1);
  if (mSelectionSkipCount < 1)
  {
    mSelectionSkipCount = 1;
    ROS_WARN("Skip Count will be set to 1.");
  }

  mNh.param<int>("timeDiff", mSelectionTimeMS, 1000);
  if (mSelectionTimeMS < 1)
  {
    mSelectionTimeMS = 1000;
    ROS_WARN("Time difference will be set to 1000 ms.");
  }

  mNh.param<int>("baseline", mSelectionBaseline, 1);
  if (mSelectionBaseline < 1)
  {
    mSelectionBaseline = 1;
    ROS_WARN("Required Baseline will be set to 1 m.");
  }

  /*mNh.param<int>("bufferSize", mBufferSize, 100);
  if (mBufferSize < 1)
  {
    mBufferSize = 100;
    ROS_WARN("Buffer size will be set to 100.");
  }*/

  mNh.param<int>("verbosity_level", tmp, 7);
  if (tmp != mVerbosityLevel)
  {
    mVerbosityLevel = tmp;
  }

  mNh.param<double>("aphaRect", mRectAlpha, 0.2);

  mNh.param<std::string>("featureDetector", text, "");
  auto it = find(mAvailDetector.begin(), mAvailDetector.end(), text);
  if (it != mAvailDetector.end())
  {
    mChosenDetector = std::distance(mAvailDetector.begin(), it);

  }

  mNh.param<std::string>("descriptorExtractor", text, "");
  it = find(mAvailExtractor.begin(), mAvailExtractor.end(), text);
  if (it != mAvailExtractor.end())
  {
    mChosenExtractor = std::distance(mAvailExtractor.begin(), it);

  }

  mNh.param<std::string>("matchAlg", text, "GMBSOF");
  it = find(mAvailMatcher.begin(), mAvailMatcher.end(), text);
  if (it != mAvailMatcher.end())
  {
    mChosenMatcher = std::distance(mAvailMatcher.begin(), it);
  }

  // --> remaining settings
  mNh.param<bool>("bNoRatioTest",   mbNoRatioTest,   false);
  mNh.param<bool>("bRefineVFC",     mbRefineVFC,     false);
  mNh.param<bool>("bRefineSOF",     mbRefineSOF,     false);
  mNh.param<bool>("bRefineGMS",     mbRefineGMS,     false);
  mNh.param<bool>("bDynKeyP",       mbDynKeyP,       false);
  mNh.param<bool>("bAutoAdaptTH",   mbAutoAdaptTH,   false);

  mNh.param<int>("refineSubPx",  tmp,   0);
  if (tmp != mRefineSubPx && tmp < 3 && tmp >= 0)
  {
    mRefineSubPx = tmp;
  }

  mNh.param<int>("refineRT",      tmp,      0);
  if (tmp != mRefineRT && tmp < 7 && tmp >= 0)
  {
    mRefineRT = tmp;
    set_refineRT_config(0, mRefineRT, mbRefineRTold, mbkneipInsteadBA, mRefineMethod);
  }
  mNh.param<int>("refineRTweights",      tmp,      2);
  if (tmp != mRefineRTweights && tmp < 3 && tmp >= 0)
  {
    mRefineRTweights = tmp;
    set_refineRT_config(1, mRefineRTweights, mbRefineRTold, mbkneipInsteadBA, mRefineMethod);
  }

  mNh.param<double>("th", mth, 0.8);

  if ((mRefineSubPx != 1) && (mth < 1.2))
      mth = 1.2;

  mNh.param<int>("maxKpPerFrame", mMaxKp, 2000);
  mNh.param<bool>("bPublishMatches", mbPublishMatches, true);
  mNh.param<int>("numDrawnMatches", mNumDrawnMatches, 50);
  mNh.param<int>("estimateHomography", mEstimateHomography, 0);
  mNh.param<int>("refineBA", mRefineBA, 0);

  mNh.param<std::string>("robustMethod", text, "");
  it = find(mAvailRobustMethod.begin(), mAvailRobustMethod.end(), text);
  if (it != mAvailRobustMethod.end())
  {
    mChosenRobustMethod = std::distance(mAvailRobustMethod.begin(), it);
  }

  mNh.param<std::string>("nmsQry", mNmsQuery, "");
  mNh.param<std::string>("nmsIdx", mNmsIndex, "");

  mNh.param<bool>("bHistEqualization", mbHistEqualization, false);

  mNh.param<int>("usacSPRT", mUsacSel[0], 3);
  set_usac_config(0, mUsacSel[0]);
  mNh.param<int>("usacPROSACbeta", mUsacSel[1], 1);
  set_usac_config(1, mUsacSel[1]);
  mNh.param<int>("usacPrevalidateSamples", mUsacSel[2], 1);
  set_usac_config(2, mUsacSel[2]);
  mNh.param<int>("usacDegeneracyCheck", mUsacSel[3], 2);
  set_usac_config(3, mUsacSel[3]);
  mNh.param<int>("usacEstimator", mUsacSel[4], 2);
  set_usac_config(4, mUsacSel[4]);
  mNh.param<int>("usacInnerRefinementAlgorithm", mUsacSel[5], 5);
  set_usac_config(5, mUsacSel[5]);
  mNh.param<double>("usacDegeneracyThreshold", mUsacTh, 0.85);
  mUsacConfig.degenDecisionTh = mUsacTh;
  mUsacConfig.th_pixels = mth;
  mNh.param<int>("usacInlratFilt", mUSACInlratFilt, 0);

  mNh.param<double>("maxDist3DPtsZ", mMaxDist3DPtsZ, 50.0);

  if ((mMaxDist3DPtsZ < 5.0) || (mMaxDist3DPtsZ > 200.0))
      mMaxDist3DPtsZ = 50.0;


  //Parameters for stereo refinement
  bool btmp = false;
  mNh.param<bool>("bStereoRef", btmp, false);
  if ((mType != k_stereo) && btmp)
  {
      ROS_WARN("For stereo refinement, stereo data must be available and the input type in the yaml-File set to stereo! Using mono configuration!");
  }
  else
  {
      if(btmp)
        newStereoRefEnable = true;
      mbStereoRef = btmp;
  }
  mNh.param<int>("evStepStereoStable", mEvStepStereoStable, 0);
  mNh.param<bool>("useOnlyStablePose", mbUseOnlyStablePose, false);
  mNh.param<bool>("useMostLikelyPose", mbUseMostLikelyPose, false);
  mNh.param<int>("refineRT_stereo",      tmp,      4);
  if (tmp != mRefineRT_stereo && tmp < 7 && tmp >= 0)
  {
    mRefineRT_stereo = tmp;
    set_refineRT_config(0, mRefineRT_stereo, mbRefineRTold_stereo, mbkneipInsteadBA_stereo, mRefineMethod_stereo);
  }
  mNh.param<int>("refineRTweights_stereo",      tmp,      2);
  if (tmp != mRefineRTweights_stereo && tmp < 3 && tmp >= 0)
  {
    mRefineRTweights_stereo = tmp;
    set_refineRT_config(1, mRefineRTweights_stereo, mbRefineRTold_stereo, mbkneipInsteadBA_stereo, mRefineMethod_stereo);
  }
  mNh.param<int>("refineBA_stereo", mRefineBA_stereo, 0);
  mNh.param<double>("minStartAggInlRat", mMinStartAggInlRat, 0.2);
  mNh.param<double>("relInlRatThLast", mRelInlRatThLast, 0.35);
  mNh.param<double>("relInlRatThNew", mRelInlRatThNew, 0.2);
  mNh.param<double>("minInlierRatSkip", mMinInlierRatSkip, 0.38);
  mNh.param<double>("relMinInlierRatSkip", mRelMinInlierRatSkip, 0.7);
  //mNh.param<size_t>("maxSkipPairs", mMaxSkipPairs, 5);
  _param_get_sizet(mMaxSkipPairs, "maxSkipPairs", 5);
  mNh.param<double>("minInlierRatioReInit", mMinInlierRatioReInit, 0.6);
  mNh.param<float>("minPtsDistance", mMinPtsDistance, 3.f);
  //mNh.param<size_t>("maxPoolCorrespondences", mMaxPoolCorrespondences, 30000);
  _param_get_sizet(mMaxPoolCorrespondences, "maxPoolCorrespondences", 30000);
  //mNh.param<size_t>("minContStablePoses", mMinContStablePoses, 3);
  _param_get_sizet(mMinContStablePoses, "minContStablePoses", 3);
  mNh.param<double>("absThRankingStable", mAbsThRankingStable, 0.075);
  mNh.param<bool>("useRANSAC_fewMatches", mbUseRANSAC_fewMatches, false);
  //mNh.param<size_t>("checkPoolPoseRobust", mCheckPoolPoseRobust, 3);
  _param_get_sizet(mCheckPoolPoseRobust, "checkPoolPoseRobust", 3);
  mNh.param<double>("minNormDistStable", mMinNormDistStable, 0.5);
  mNh.param<int>("raiseSkipCntIncr", mRaiseSkipCntIncr, 0);
  mNh.param<int>("raiseSkipCntStable", mRaiseSkipCntStable, 0);
  mNh.param<double>("maxRat3DPtsFar", mMaxRat3DPtsFar, 0.5);
  if(mbStereoRef)
  {
      set_Stereo_Refine_Options();
      evStepStereoStable_tmp = mEvStepStereoStable + 1;
  }


  init_dynamic_config();
}

void ROSnode::init_dynamic_config()
{
  mServer.getConfigDefault(mDynConfig);

  mDynConfig.feature_detector = mChosenDetector;
  mDynConfig.descriptor_extractor = mChosenExtractor;
  mDynConfig.matcher = mChosenMatcher;

  mDynConfig.disable_ratio_test = mbNoRatioTest;
  mDynConfig.refine_VFC = mbRefineVFC;
  mDynConfig.refine_SOF = mbRefineSOF;
  mDynConfig.refine_GMS = mbRefineGMS;
  mDynConfig.detect_keypoints_dynamically = mbDynKeyP;//was commented
  mDynConfig.refine_sub_px = mRefineSubPx;
  mDynConfig.auto_adapt_threshold = mbAutoAdaptTH;
  mDynConfig.refine_RT = mRefineRT;
  mDynConfig.refine_RT_weights = mRefineRTweights;

  if ((mRefineSubPx != 1) && (mth < 1.2))
  {
      mth = 1.2;
      mDynConfig.sampson_error_threshold = mth;
  }
  else
      mDynConfig.sampson_error_threshold = mth;

  mDynConfig.verbosity_level = mVerbosityLevel;

  mDynConfig.rect_img_scaling = mRectAlpha;

  mDynConfig.max_keypoints_per_frame = mMaxKp;

  mDynConfig.publish_matches = mbPublishMatches;
  mDynConfig.num_drawn_matches = mNumDrawnMatches;

  mDynConfig.estimate_homographies = mEstimateHomography;
  mDynConfig.refine_BA = mRefineBA;
  mDynConfig.robust_method = mChosenRobustMethod;

  mDynConfig.nms_idx = mNmsIndex;
  mDynConfig.nms_query = mNmsQuery;
  mDynConfig.hist_equalization = mbHistEqualization;

  mDynConfig.sprt = mUsacSel[0];
  mDynConfig.prosac_beta = mUsacSel[1];
  mDynConfig.prevalidate_samples = mUsacSel[2];
  mDynConfig.degeneracy_check = mUsacSel[3];
  mDynConfig.degeneracy_threshold = mUsacTh;
  mDynConfig.usac_estimator = mUsacSel[4];
  mDynConfig.inner_refinement_algorithm = mUsacSel[5];
  mDynConfig.usac_inlrat_filter = mUSACInlratFilt;

  mDynConfig.max_z_distance_3DPts = mMaxDist3DPtsZ;

  //Parameters for stereo refinement
  mDynConfig.stereo_refine = mbStereoRef;
  mDynConfig.eval_nth_pair_if_stable = mEvStepStereoStable;
  mDynConfig.use_only_stable_pose = mbUseOnlyStablePose;
  mDynConfig.use_most_likely_pose = mbUseMostLikelyPose;
  mDynConfig.refine_RT_stereo = mRefineRT_stereo;
  mDynConfig.refine_RT_weights_stereo = mRefineRTweights_stereo;
  mDynConfig.refine_BA_stereo = mRefineBA_stereo;
  mDynConfig.min_Inl_Rat_Start_Corr_Aggregation = mMinStartAggInlRat;
  mDynConfig.rel_inv_Inl_Rat_TH_to_Last_Inl_Rat_use_Robust = mRelInlRatThLast;
  mDynConfig.rel_inv_Inl_Rat_TH_new_Data_after_Robust = mRelInlRatThNew;
  mDynConfig.abs_max_Inl_Rat_TH_on_new_robust_for_bad_pair = mMinInlierRatSkip;
  mDynConfig.rel_Inl_Rat_TH_new_robust_last_valid_bad_pair = mRelMinInlierRatSkip;
  mDynConfig.max_skip_pairs_change_detected = static_cast<int>(mMaxSkipPairs);
  mDynConfig.min_Inl_Rat_TH_after_robust_reinit = mMinInlierRatioReInit;
  mDynConfig.min_Pts_dist_in_pool = static_cast<double>(mMinPtsDistance);
  mDynConfig.max_Pool_Correspondences = static_cast<int>(mMaxPoolCorrespondences);
  mDynConfig.min_similar_poses_to_stable = static_cast<int>(mMinContStablePoses);
  mDynConfig.max_norm_error_range_th_pairs_to_stable = mAbsThRankingStable;
  mDynConfig.use_RANSAC_few_corrs = mbUseRANSAC_fewMatches;
  mDynConfig.nth_iteration_robust_on_pool = static_cast<int>(mCheckPoolPoseRobust);
  mDynConfig.min_dorm_distance_for_stable_pose = mMinNormDistStable;
  mDynConfig.raise_skip_pair_cnt_factor = mRaiseSkipCntIncr;
  mDynConfig.raise_skip_pair_cnt_after_n_stable = mRaiseSkipCntStable;
  mDynConfig.max_ratio_3DPts_too_far = mMaxRat3DPtsFar;

  // set selection options
  switch (mSelectionType)
  {
    case k_all: mDynConfig.selection_type = 0; break;
    case k_skip: mDynConfig.selection_type = 1; break;
    case k_time: mDynConfig.selection_type = 2; break;
    case k_baseline: mDynConfig.selection_type = 3; break;
  }
  mDynConfig.selection_skip_count = mSelectionSkipCount;
  mDynConfig.selection_time_diff_ms = mSelectionTimeMS;
  mDynConfig.selection_baseline_m = mSelectionBaseline;

  mServer.updateConfig(mDynConfig);
}

void ROSnode::set_usac_config(int id, int selection)
{
  switch (id)
  {
    case 0: // SPRT
    {
            switch (selection)
            {
            case 0:
                mUsacConfig.automaticSprtInit = poselib::SprtInit::SPRT_DEFAULT_INIT;
                break;
            case 1:
                mUsacConfig.automaticSprtInit = poselib::SprtInit::SPRT_DELTA_AUTOM_INIT;
                break;
            case 2:
                mUsacConfig.automaticSprtInit = poselib::SprtInit::SPRT_EPSILON_AUTOM_INIT;
                break;
            case 3: default:
                mUsacConfig.automaticSprtInit = poselib::SprtInit::SPRT_DELTA_AUTOM_INIT | poselib::SprtInit::SPRT_EPSILON_AUTOM_INIT;
                break;
            }
    } break;
    case 1: // PROSAC beta
    {
            switch (selection)
            {
            case 0:
                mUsacConfig.noAutomaticProsacParamters = true;
                break;
            case 1: default:
                mUsacConfig.noAutomaticProsacParamters = false;
                break;
            }
    } break;
    case 2: // prevalidate_samples
    {
            switch (selection)
            {
            case 0:
                mUsacConfig.prevalidateSample = false;
                break;
            case 1: default:
                mUsacConfig.prevalidateSample = true;
                break;
            }
    } break;
    case 3: // degeneracy check
    {
            switch (selection)
            {
            case 0:
                mUsacConfig.degeneracyCheck = poselib::UsacChkDegenType::DEGEN_NO_CHECK;
                break;
            case 1:
                mUsacConfig.degeneracyCheck = poselib::UsacChkDegenType::DEGEN_QDEGSAC;
                break;
            case 2: default:
                mUsacConfig.degeneracyCheck = poselib::UsacChkDegenType::DEGEN_USAC_INTERNAL;
                break;
            }
    } break;
    case 4: // estimator
    {
            switch (selection)
            {
            case 0:
                mUsacConfig.estimator = poselib::PoseEstimator::POSE_NISTER;
                break;
            case 1:
                mUsacConfig.estimator = poselib::PoseEstimator::POSE_EIG_KNEIP;
                break;
            case 2: default:
                mUsacConfig.estimator = poselib::PoseEstimator::POSE_STEWENIUS;
                break;
            }
    } break;
    case 5: // inner refinement algorithm
    {
            switch (selection)
            {
            case 0:
                mUsacConfig.refinealg = poselib::RefineAlg::REF_WEIGHTS;
                break;
            case 1:
                mUsacConfig.refinealg = poselib::RefineAlg::REF_8PT_PSEUDOHUBER;
                break;
            case 2:
                mUsacConfig.refinealg = poselib::RefineAlg::REF_EIG_KNEIP;
                break;
            case 3:
                mUsacConfig.refinealg = poselib::RefineAlg::REF_EIG_KNEIP_WEIGHTS;
                break;
            case 4:
                mUsacConfig.refinealg = poselib::RefineAlg::REF_STEWENIUS;
                break;
            case 6:
                mUsacConfig.refinealg = poselib::RefineAlg::REF_NISTER;
                break;
            case 7:
                mUsacConfig.refinealg = poselib::RefineAlg::REF_NISTER_WEIGHTS;
                break;
            case 5: default:
                mUsacConfig.refinealg = poselib::RefineAlg::REF_STEWENIUS_WEIGHTS;
                break;
            }
    } break;
  }
}

void ROSnode::set_Stereo_Refine_Options()
{
    cfg_stereo.keypointType = mAvailDetector.at(mChosenDetector);
    cfg_stereo.descriptorType = mAvailExtractor.at(mChosenExtractor);
    cfg_stereo.th_pix_user = mth;
    cfg_stereo.autoTH = mbAutoAdaptTH;
    cfg_stereo.Halign = mEstimateHomography;
    cfg_stereo.RobMethod = mAvailRobustMethod.at(mChosenRobustMethod);
    cfg_stereo.refineMethod = mRefineMethod;
    cfg_stereo.refineRTold = mbRefineRTold;
    cfg_stereo.kneipInsteadBA = mbkneipInsteadBA;
    cfg_stereo.BART = mRefineBA;
    cfg_stereo.refineMethod_CorrPool = mRefineMethod_stereo;
    cfg_stereo.refineRTold_CorrPool = mbRefineRTold_stereo;
    cfg_stereo.kneipInsteadBA_CorrPool = mbkneipInsteadBA_stereo;
    cfg_stereo.BART_CorrPool = mRefineBA_stereo;
    cfg_stereo.verbose = mVerbosityLevel;
    cfg_stereo.minStartAggInlRat = mMinStartAggInlRat;
    cfg_stereo.relInlRatThLast = mRelInlRatThLast;
    cfg_stereo.relInlRatThNew = mRelInlRatThNew;
    cfg_stereo.minInlierRatSkip = mMinInlierRatSkip;
    cfg_stereo.relMinInlierRatSkip = mRelMinInlierRatSkip;
    cfg_stereo.maxSkipPairs = mMaxSkipPairs;
    cfg_stereo.minInlierRatioReInit = mMinInlierRatioReInit;
    cfg_stereo.minPtsDistance = mMinPtsDistance;
    cfg_stereo.maxPoolCorrespondences = mMaxPoolCorrespondences;
    cfg_stereo.minContStablePoses = mMinContStablePoses;
    cfg_stereo.absThRankingStable = mAbsThRankingStable;
    cfg_stereo.useRANSAC_fewMatches = mbUseRANSAC_fewMatches;
    cfg_stereo.checkPoolPoseRobust = mCheckPoolPoseRobust;
    cfg_stereo.minNormDistStable = mMinNormDistStable;
    cfg_stereo.maxDist3DPtsZ = mMaxDist3DPtsZ;
    cfg_stereo.raiseSkipCnt = (mRaiseSkipCntIncr | (mRaiseSkipCntStable << 4));
    cfg_stereo.maxRat3DPtsFar = mMaxRat3DPtsFar;
}

void ROSnode::configureUSAC(cv::Size imgSize,
    std::vector<cv::KeyPoint> *kp0,
    std::vector<cv::KeyPoint> *kp1,
    std::vector<cv::DMatch> *finalMatches)
{
    // setup USAC
    mUsacConfig.focalLength = (mK0.at<double>(0, 0) + mK0.at<double>(1, 1) + mK1.at<double>(0, 0) + mK1.at<double>(1, 1)) / 4;
    mUsacConfig.imgSize = imgSize;
    mUsacConfig.keypoints1 = kp0;
    mUsacConfig.keypoints2 = kp1;
    mUsacConfig.matches = finalMatches;

    if ((mUsacConfig.automaticSprtInit & poselib::SprtInit::SPRT_EPSILON_AUTOM_INIT) > 0)
    {
      std::vector<cv::DMatch> vfcfilteredMatches;
      int err = 0;
      unsigned int n_f = 0;
      if (mUSACInlratFilt == 0)
      {
          n_f = (unsigned int)filterMatchesGMS(*kp0, imgSize, *kp1, imgSize, *finalMatches, vfcfilteredMatches);
          if (n_f == 0)
              err = -1;
      }
      else
      {
          err = filterWithVFC(*kp0, *kp1, *finalMatches, vfcfilteredMatches);
          n_f = (unsigned int)vfcfilteredMatches.size();
      }

      if (!err)
      {
        if ((8 < n_f) || (finalMatches->size() < 24))
        {
          mUsacConfig.nrMatchesVfcFiltered = n_f;
        }
        else if (mUsacConfig.automaticSprtInit == poselib::SprtInit::SPRT_EPSILON_AUTOM_INIT)
        {
          mUsacConfig.automaticSprtInit = poselib::SprtInit::SPRT_DEFAULT_INIT;
        }
        else
        {
          mUsacConfig.automaticSprtInit = poselib::SprtInit::SPRT_DELTA_AUTOM_INIT;
        }
      }
      else if (mUsacConfig.automaticSprtInit == poselib::SprtInit::SPRT_EPSILON_AUTOM_INIT)
      {
        mUsacConfig.automaticSprtInit = poselib::SprtInit::SPRT_DEFAULT_INIT;
      }
      else
      {
        mUsacConfig.automaticSprtInit = poselib::SprtInit::SPRT_DELTA_AUTOM_INIT;
      }
    }
}

void ROSnode::set_refineRT_config(int id, int selection, bool &refineRTold_, bool &kneipInsteadBA_, int &refineMethod_)
{
    switch (id)
    {
      case(0): // Refinment algorithm
      {
          refineRTold_ = false;
          kneipInsteadBA_ = false;
          switch (selection)
          {
          case(0):
              refineMethod_ = poselib::RefinePostAlg::PR_NO_REFINEMENT;
              break;
          case(1):
              refineMethod_ = poselib::RefinePostAlg::PR_NO_REFINEMENT;
              refineRTold_ = true;
              break;
          case(2):
              refineMethod_ = poselib::RefinePostAlg::PR_8PT;
              break;
          case(3):
              refineMethod_ = poselib::RefinePostAlg::PR_NISTER;
              break;
          case(4):
              refineMethod_ = poselib::RefinePostAlg::PR_STEWENIUS;
              break;
          case(5):
              refineMethod_ = poselib::RefinePostAlg::PR_KNEIP;
              break;
          case(6):
              refineMethod_ = poselib::RefinePostAlg::PR_KNEIP;
              kneipInsteadBA_ = true;
              break;
          default:
              refineMethod_ = poselib::RefinePostAlg::PR_NO_REFINEMENT;
              break;
          }
      } break;
      case(1): // Weighting function
      {
          switch (selection)
          {
          case(0):
              refineMethod_ = (refineMethod_ | poselib::RefinePostAlg::PR_NO_WEIGHTS);
              break;
          case(1):
              refineMethod_ = (refineMethod_ | poselib::RefinePostAlg::PR_TORR_WEIGHTS);
              break;
          case(2): default:
              refineMethod_ = (refineMethod_ | poselib::RefinePostAlg::PR_PSEUDOHUBER_WEIGHTS);
              break;
          }
      } break;
    }
}

bool ROSnode::update_()
{
  mChosenDetector     = mDynConfig.feature_detector;
  mChosenExtractor    = mDynConfig.descriptor_extractor;

  if (!mbNoRatioTest && mAvailMatcher.at(mDynConfig.matcher) == "GMBSOF")
  {
      ROS_WARN("Ratio test must be enabled when choosing GMBSOF!");
    return false;
  }

  mChosenMatcher      = mDynConfig.matcher;

  if (mAvailRobustMethod.at(mDynConfig.robust_method) != "ARRSAC" && (mEstimateHomography || mbAutoAdaptTH))
  {
      if(mEstimateHomography)
          ROS_WARN("Homography alignment is enabled and only works in cunjuction with ARRSAC! To choose a different robust method, you must deselect homography alignment first.");
      if(mbAutoAdaptTH)
          ROS_WARN("The automatic threshold estimation is only available if ARRSAC is selected! To choose a different robust method, you must deselect the automatic threshold estimation first.");
    return false;
  }
  else
  {
    mChosenRobustMethod = mDynConfig.robust_method;
  }

  if (mAvailMatcher.at(mChosenMatcher) == "GMBSOF" && mDynConfig.disable_ratio_test)
  {
    return false;
  }

  mbNoRatioTest       = mDynConfig.disable_ratio_test;
  mbRefineVFC         = mDynConfig.refine_VFC;
  mbRefineSOF         = mDynConfig.refine_SOF;
  mbRefineGMS         = mDynConfig.refine_GMS;
  mRefineSubPx       = mDynConfig.refine_sub_px;
  mbDynKeyP           = mDynConfig.detect_keypoints_dynamically;
  if (mDynConfig.auto_adapt_threshold && (mEstimateHomography || (mAvailRobustMethod.at(mChosenRobustMethod) != "ARRSAC")))
  {
      if(mEstimateHomography)
          ROS_WARN("The automatic threshold estimation is not compatible with homography alignment! Deselect homography alignment first.");
      if(mAvailRobustMethod.at(mChosenRobustMethod) != "ARRSAC")
          ROS_WARN("The automatic threshold estimation only works in cunjuction with ARRSAC!! Select ARRSAC first.");
    return false;
  }
  else
  {
    mbAutoAdaptTH       = mDynConfig.auto_adapt_threshold;
  }
  mRefineRT          = mDynConfig.refine_RT;
  set_refineRT_config(0, mRefineRT, mbRefineRTold, mbkneipInsteadBA, mRefineMethod);
  mRefineRTweights   = mDynConfig.refine_RT_weights;
  set_refineRT_config(1, mRefineRTweights, mbRefineRTold, mbkneipInsteadBA, mRefineMethod);

  mth                = mDynConfig.sampson_error_threshold;
  if ((mRefineSubPx != 1) && (mth < 1.2))
  {
      mth = 1.2;
      ROS_WARN("The selected threshold is too small for estimations without using subpixel refinement with template matching! Setting it to 1.2.");
      mDynConfig.sampson_error_threshold = mth;
  }

  mVerbosityLevel     = mDynConfig.verbosity_level;
  mRectAlpha          = mDynConfig.rect_img_scaling;
  mMaxKp              = mDynConfig.max_keypoints_per_frame;

  mbPublishMatches    = mDynConfig.publish_matches;
  mNumDrawnMatches    = mDynConfig.num_drawn_matches;

  if (mDynConfig.estimate_homographies && (mbAutoAdaptTH  || (mAvailRobustMethod.at(mChosenRobustMethod) != "ARRSAC")))
  {
      if(mbAutoAdaptTH)
          ROS_WARN("The homography alignment is not compatible with the automatic threshold estimation! Deselect automatic threshold estimation first.");
      if(mAvailRobustMethod.at(mChosenRobustMethod) != "ARRSAC")
          ROS_WARN("The homography alignment only works in cunjuction with ARRSAC!! Select ARRSAC first.");
    return false;
  }
  else
  {
    mEstimateHomography = mDynConfig.estimate_homographies;
  }

  mRefineBA           = mDynConfig.refine_BA;

  if (mDynConfig.sprt > 1 && (mbRefineVFC || mbRefineGMS))
  {
    ROS_WARN("Estimate of epsilon for SPRT might be wrong as refine_VFC or refine_GMS is enabled.");
    if(mbRefineVFC)
    {
        mbRefineVFC = false;
        mbRefineVFC_tmp = true;
        mbRefineGMS_tmp = false;
    }
    else
    {
        mbRefineGMS = false;
        mbRefineGMS_tmp = true;
        mbRefineVFC_tmp = false;
    }
  }
  else
  {
    mbRefineGMS_tmp = false;
    mbRefineVFC_tmp = false;
  }
  set_usac_config(0, mDynConfig.sprt);
  set_usac_config(1, mDynConfig.prosac_beta);
  set_usac_config(2, mDynConfig.prevalidate_samples);
  set_usac_config(3, mDynConfig.degeneracy_check);
  set_usac_config(4, mDynConfig.usac_estimator);
  set_usac_config(5, mDynConfig.inner_refinement_algorithm);

  mUsacConfig.degenDecisionTh = mDynConfig.degeneracy_threshold;
  mUsacConfig.th_pixels = mth;
  mUSACInlratFilt = mDynConfig.usac_inlrat_filter;

  mNmsQuery = mDynConfig.nms_query;
  mNmsIndex  = mDynConfig.nms_idx;

  mbHistEqualization = mDynConfig.hist_equalization;

  mMaxDist3DPtsZ = mDynConfig.max_z_distance_3DPts;

  //Parameters for stereo refinement
  if ((mType != k_stereo) && mDynConfig.stereo_refine)
  {
      ROS_WARN("For stereo refinement, stereo data must be available and the input type in the yaml-File set to stereo! Using mono configuration!");
      mbStereoRef = false;
      mDynConfig.stereo_refine = mbStereoRef;
  }
  else
  {
      if((mbStereoRef != mDynConfig.stereo_refine) && mDynConfig.stereo_refine)
          newStereoRefEnable = true;
      else if((mbStereoRef != mDynConfig.stereo_refine) && !mDynConfig.stereo_refine && stereoObj)
          stereoObj.release();
      mbStereoRef = mDynConfig.stereo_refine;
  }
  mEvStepStereoStable = mDynConfig.eval_nth_pair_if_stable;
  mbUseOnlyStablePose = mDynConfig.use_only_stable_pose;
  mbUseMostLikelyPose = mDynConfig.use_most_likely_pose;
  mRefineRT_stereo = mDynConfig.refine_RT_stereo;
  set_refineRT_config(0, mRefineRT_stereo, mbRefineRTold_stereo, mbkneipInsteadBA_stereo, mRefineMethod_stereo);
  mRefineRTweights_stereo = mDynConfig.refine_RT_weights_stereo;
  set_refineRT_config(1, mRefineRTweights_stereo, mbRefineRTold_stereo, mbkneipInsteadBA_stereo, mRefineMethod_stereo);
  mRefineBA_stereo = mDynConfig.refine_BA_stereo;
  if(!mbRefineRTold_stereo && !mbkneipInsteadBA_stereo && !mRefineBA_stereo && ((mRefineMethod_stereo & 0xF) == poselib::RefinePostAlg::PR_NO_REFINEMENT))
  {
      ROS_WARN("For stereo refinement, some sort of refinement must be enabled! Using Stewenius with Pseudo-Huber weights.");
      mRefineMethod_stereo = poselib::RefinePostAlg::PR_STEWENIUS | poselib::RefinePostAlg::PR_PSEUDOHUBER_WEIGHTS;
      mRefineRT_stereo = 4;
      mRefineRTweights_stereo = 2;
      mDynConfig.refine_RT_stereo = mRefineRT_stereo;
      mDynConfig.refine_RT_weights_stereo = mRefineRTweights_stereo;
  }
  mMinStartAggInlRat = mDynConfig.min_Inl_Rat_Start_Corr_Aggregation;
  mRelInlRatThLast = mDynConfig.rel_inv_Inl_Rat_TH_to_Last_Inl_Rat_use_Robust;
  mRelInlRatThNew = mDynConfig.rel_inv_Inl_Rat_TH_new_Data_after_Robust;
  mMinInlierRatSkip = mDynConfig.abs_max_Inl_Rat_TH_on_new_robust_for_bad_pair;
  mRelMinInlierRatSkip = mDynConfig.rel_Inl_Rat_TH_new_robust_last_valid_bad_pair;
  mMaxSkipPairs = static_cast<size_t>(mDynConfig.max_skip_pairs_change_detected);
  mMinInlierRatioReInit = mDynConfig.min_Inl_Rat_TH_after_robust_reinit;
  mMinPtsDistance = static_cast<float>(mDynConfig.min_Pts_dist_in_pool);
  mMaxPoolCorrespondences = static_cast<size_t>(mDynConfig.max_Pool_Correspondences);
  mMinContStablePoses = static_cast<size_t>(mDynConfig.min_similar_poses_to_stable);
  mAbsThRankingStable = mDynConfig.max_norm_error_range_th_pairs_to_stable;
  mbUseRANSAC_fewMatches = mDynConfig.use_RANSAC_few_corrs;
  mCheckPoolPoseRobust = static_cast<size_t>(mDynConfig.nth_iteration_robust_on_pool);
  mMinNormDistStable = mDynConfig.min_dorm_distance_for_stable_pose;
  mRaiseSkipCntIncr = mDynConfig.raise_skip_pair_cnt_factor;
  mRaiseSkipCntStable = mDynConfig.raise_skip_pair_cnt_after_n_stable;
  mMaxRat3DPtsFar = mDynConfig.max_ratio_3DPts_too_far;
  if(mbStereoRef)
  {
      set_Stereo_Refine_Options();
      evStepStereoStable_tmp = mEvStepStereoStable + 1;
      mbNewSettings = true;
  }

  if (mDynConfig.selection_type == 0)
  {
    mSelectionType = k_all;
  }
  else if (mDynConfig.selection_type == 1)
  {
    mSelectionType = k_skip;
    mSelectionSkipCount = mDynConfig.selection_skip_count;
  }
  else if (mDynConfig.selection_type == 2)
  {
    mSelectionType = k_time;
    mSelectionTimeMS = mDynConfig.selection_time_diff_ms;
  }
  else
  {
    mSelectionType = k_baseline;
    mSelectionBaseline = mDynConfig.selection_baseline_m;
  }

  return true;
}

void ROSnode::init()
{
}


// is called when mHasNewMessage == true
void ROSnode::process()
{

  if (!mbHasCameraInfo)
  {
    ROS_WARN("CameraInfo not available.");
    return;
  }

  int err;
  double sumSqrErr = 0.;
  double confidence = 0.;
  cv::Mat img0, img1;
  cv::Mat R0, t0;

   if (mType == k_mono)
  {
    if (!select_images_mono(img0, R0, t0, img1, mR1, mt1))
    {
      ROS_WARN("No images available.");
      return;
    }
  }
  else
  {
    // check if new images (with time diff or skip diff) are available
    if (!select_images_stereo(img0, R0, t0, img1, mR1, mt1))
    {
      ROS_WARN("No images available.");
      return;
    }
    if(newStereoRefEnable)
    {
        evStepStereoStable_cnt = evStepStereoStable_tmp;
        newStereoRefEnable = false;
        poseWasStable = false;
    }
    if(mbNewSettings && cfg_stereo.dist0_8 && cfg_stereo.dist1_8 && cfg_stereo.K0 && cfg_stereo.K1)
    {
      mbNewSettings = false;
      if(stereoObj)
      {
          stereoObj->setNewParameters(cfg_stereo);
      }
      else
      {
          stereoObj.reset(new poselib::StereoRefine(cfg_stereo));
      }
    }
  }


  cv::Mat R, t;
  // pose estimation
  if (!mbSkipPoseEst)
  {
    std::vector<cv::DMatch> finalMatches;
    std::vector<cv::KeyPoint> kp0, kp1;
    double time = 0, time_total = 0;

    if (!mbStereoRef || (evStepStereoStable_cnt == evStepStereoStable_tmp) || (evStepStereoStable_cnt == 0))
    {
        if (mbHistEqualization)
        {
          equalizeHist(img0, img0);
          equalizeHist(img1, img1);
        }

        //Matching
        err = matchinglib::getCorrespondences(img0, img1, finalMatches, kp0, kp1, mAvailDetector.at(mChosenDetector), mAvailExtractor.at(mChosenExtractor), mAvailMatcher.at(mChosenMatcher), mbDynKeyP, mMaxKp, mbRefineVFC, mbRefineGMS, !mbNoRatioTest, mbRefineSOF, mRefineSubPx, ((mVerbosityLevel < 4) || (5 < mVerbosityLevel)) ? mVerbosityLevel : 0);
        if (err)
        {
          ROS_WARN("Matching failed.");
          return;
        }

        // setup USAC
        configureUSAC(img0.size(), &kp0, &kp1, &finalMatches);

        //Filter matches with VFC or GMS if automatic epsilon estimation for SPRT in USAC is enabled and, thus, filtering was not possible before (as one of the filtering techniques is used to estimate the inlier ratio when configuring USAC)
        if(mbRefineGMS_tmp)
        {
            std::vector<bool> inlierMaskGMS;
            int n_gms = filterMatchesGMS(kp0, img0.size(),	kp1, img1.size(), finalMatches, inlierMaskGMS, false, false);

            if ((n_gms > 50) || ((double)n_gms / (double)finalMatches.size() > 0.5))
            {
                std::vector<cv::DMatch> matches_tmp(n_gms);
                for (size_t i = 0, j = 0; i < inlierMaskGMS.size(); i++)
                {
                    if (inlierMaskGMS[i])
                        matches_tmp[j++] = finalMatches[i];
                }
                finalMatches = matches_tmp;
            }
        }
        else if(mbRefineVFC_tmp)
        {
            std::vector<cv::DMatch> vfcfilteredMatches;
            if(!filterWithVFC(kp0, kp1, finalMatches, vfcfilteredMatches))
            {
                if ((vfcfilteredMatches.size() > 50) || ((double)vfcfilteredMatches.size() / (double)finalMatches.size() > 0.5))
                {
                    finalMatches = vfcfilteredMatches;
                }
            }
        }

        if (mbPublishMatches)
        {
          cv::Mat outMatches;
          draw_matches(img0, img1, kp0, kp1, finalMatches, outMatches);

          publish_matches(outMatches);
        }
    }

    /** Pose estimation **/
    if (!mbStereoRef)
    {
        // extract keypoints
        std::vector<cv::Point2f> pt0, pt1;
        for (size_t i = 0; i < finalMatches.size(); ++i)
        {
          pt0.push_back(kp0.at(finalMatches.at(i).queryIdx).pt);
          pt1.push_back(kp1.at(finalMatches.at(i).trainIdx).pt);
        }

        time = (double) cv::getTickCount(); //Start time measurement

        //Transfer into camera coordinates
        poselib::ImgToCamCoordTrans(pt0, mK0);
        poselib::ImgToCamCoordTrans(pt1, mK1);

        // undistort
        if (!poselib::Remove_LensDist(pt0, pt1, mDist0_8, mDist1_8))
        {
          ROS_WARN("Undistortion failed.");
          return;
        }

        if (5 < mVerbosityLevel)
        {
          time = 1000 * ((double) cv::getTickCount() - time) / cv::getTickFrequency();//End time measurement
          time_total += time;

          ROS_INFO("Time for coordinate conversion & undistortion (2 imgs): %2.2f ms", time);
        }

        time = (double) cv::getTickCount();

        // essential matrix
        cv::Mat E, mask;
        cv::Mat R_kneip = cv::Mat::eye(3,3,CV_64FC1);
        cv::Mat t_kneip = cv::Mat::zeros(3,1,CV_64FC1);
        cv::Mat p0 = cv::Mat(pt0.size(), 2, CV_64FC1);
        cv::Mat p1 = cv::Mat(pt1.size(), 2, CV_64FC1);
        double th = mth * mPxToCamFact;//Inlier threshold
        for (size_t i = 0; i < pt0.size(); ++i)
        {
          p0.at<double>(i, 0) = (double) pt0.at(i).x;
          p0.at<double>(i, 1) = (double) pt0.at(i).y;

          p1.at<double>(i, 0) = (double) pt1.at(i).x;
          p1.at<double>(i, 1) = (double) pt1.at(i).y;
        }

        if (mbAutoAdaptTH)
        {
          int inlierPoints;
          poselib::AutoThEpi EautoTH (mPxToCamFact);
          if (EautoTH.estimateEVarTH(p0, p1, E, mask, &th, &inlierPoints) != 0)
          {
            ROS_WARN("Estimation of essential matrix failed.");
            return;
          }

          ROS_INFO("Estimated threshold: %2.2f pixels", th / mPxToCamFact);
        }
        else if (mEstimateHomography)
        {
          int inliers;
          if (poselib::estimatePoseHomographies(p0, p1, R, t, E, th, inliers, mask, false, (mEstimateHomography > 1)? true : false) != 0)
          {
            ROS_WARN("Homography alignment failed.");
            return;
          }
        }
        else
        {
          if (mAvailRobustMethod.at(mChosenRobustMethod) == "USAC")
          {
            bool isDegenerate = false;
            cv::Mat R_degenerate, inliers_degenerate_R;
            bool usacError = false;
            if (mUsacConfig.refinealg == poselib::RefineAlg::REF_EIG_KNEIP || mUsacConfig.refinealg == poselib::RefineAlg::REF_EIG_KNEIP_WEIGHTS)
            {
              if (estimateEssentialOrPoseUSAC(p0, p1, E, th, mUsacConfig, isDegenerate, mask, R_degenerate, inliers_degenerate_R, R_kneip, t_kneip) != 0)
              {
                usacError = true;
              }
            }
            else
            {
              if (estimateEssentialOrPoseUSAC(p0, p1, E, th, mUsacConfig, isDegenerate, mask, R_degenerate, inliers_degenerate_R) != 0)
              {
                usacError = true;
              }
            }

            if (usacError)
            {
              ROS_WARN("USAC failed.");
              return;
            }

            if (isDegenerate)
            {
              ROS_WARN("Camera configuration is degenerate -> rotation only.");
              cv::Mat t_zero = cv::Mat::zeros(3,1,CV_64FC1);
              publish_pose(R_degenerate, t_zero);
              return;
            }
          }
          else
          {
            if (!poselib::estimateEssentialMat(E, p0, p1, mAvailRobustMethod.at(mChosenRobustMethod), th, mbRefineRTold, mask))
            {
              ROS_WARN("Estimation of essential matrix failed.");
              return;
            }
          }

        }

        // Get R, t
        cv::Mat Q;
        bool availableRT = false;
        if (mEstimateHomography)
        {
            R_kneip = R;
            t_kneip = t;
        }
        if (mEstimateHomography ||
            ((mAvailRobustMethod.at(mChosenRobustMethod) == "USAC") && (mUsacConfig.refinealg == poselib::RefineAlg::REF_EIG_KNEIP ||
                mUsacConfig.refinealg == poselib::RefineAlg::REF_EIG_KNEIP_WEIGHTS)))
        {
            double sumt = 0;
            for (int i = 0; i < 3; i++)
            {
                sumt += t_kneip.at<double>(i);
            }
            if (!poselib::nearZero(sumt) && poselib::isMatRoationMat(R_kneip))
            {
                availableRT = true;
            }
        }

        size_t nr_inliers;//= cv::countNonZero(mask);
        if(mEstimateHomography && ((mRefineMethod & 0xF) == poselib::RefinePostAlg::PR_NO_REFINEMENT))
        {
          poselib::triangPts3D(R, t, p0, p1, Q, mask, mMaxDist3DPtsZ);
        }
        else
        {
            if (mbRefineRTold)
            {
                poselib::robustEssentialRefine(p0, p1, E, E, th / 10.0, 0, true, NULL, &sumSqrErr, cv::noArray(), mask, 0);
                confidence = sumSqrErr / cv::countNonZero(mask);
                availableRT = false;
            }
            else if (((mRefineMethod & 0xF) != poselib::RefinePostAlg::PR_NO_REFINEMENT) && !mbkneipInsteadBA)
            {
                cv::Mat R_tmp, t_tmp;
                if (availableRT)
                {
                    R_kneip.copyTo(R_tmp);
                    t_kneip.copyTo(t_tmp);

                    if (poselib::refineEssentialLinear(p0, p1, E, mask, mRefineMethod, nr_inliers, R_tmp, t_tmp, th, 4, 2.0, 0.1, 0.15))
                    {
                        if (!R_tmp.empty() && !t_tmp.empty())
                        {
                            R_tmp.copyTo(R_kneip);
                            t_tmp.copyTo(t_kneip);
                        }
                    }
                    else
                        ROS_WARN("Refinement failed!");
                }
                else if ((mRefineMethod & 0xF) == poselib::RefinePostAlg::PR_KNEIP)
                {

                    if (poselib::refineEssentialLinear(p0, p1, E, mask, mRefineMethod, nr_inliers, R_tmp, t_tmp, th, 4, 2.0, 0.1, 0.15))
                    {
                        if (!R_tmp.empty() && !t_tmp.empty())
                        {
                            R_tmp.copyTo(R_kneip);
                            t_tmp.copyTo(t_kneip);
                            availableRT = true;
                        }
                        else
                            ROS_WARN("Refinement failed!");
                    }
                }
                else
                {
                    if (!poselib::refineEssentialLinear(p0, p1, E, mask, mRefineMethod, nr_inliers, cv::noArray(), cv::noArray(), th, 4, 2.0, 0.1, 0.15))
                        ROS_WARN("Refinement failed!");
                }
            }

          if (!availableRT)
          {
            poselib::getPoseTriangPts(E, p0, p1, R, t, Q, mask, mMaxDist3DPtsZ);
          }
          else
          {
            R = R_kneip;
            t = t_kneip;

            if ((mRefineBA > 0) && !mbkneipInsteadBA)
            {
              poselib::triangPts3D(R, t, p0, p1, Q, mask, mMaxDist3DPtsZ);
            }
          }
        }

        if (3 < mVerbosityLevel)
        {
          time = 1000 * ((double) cv::getTickCount() - time) / cv::getTickFrequency();
          time_total +=time;

          ROS_INFO("Time for pose estimation (includes possible linear refinement): %2.2f ms", time);
        }

        time = (double) cv::getTickCount();

        bool useBA = true;
        if (mbkneipInsteadBA)
        {
            cv::Mat R_tmp, t_tmp;
            R.copyTo(R_tmp);
            t.copyTo(t_tmp);
            bool kneipSuccess = true;
            if (poselib::refineEssentialLinear(p0, p1, E, mask, mRefineMethod, nr_inliers, R_tmp, t_tmp, th, 4, 2.0, 0.1, 0.15))
            {
                if (!R_tmp.empty() && !t_tmp.empty())
                {
                    R_tmp.copyTo(R);
                    t_tmp.copyTo(t);
                    useBA = false;
                }
                else
                    kneipSuccess = false;
            }
            else
                kneipSuccess = false;

            if(!kneipSuccess)
            {
                ROS_WARN("Refinement using Kneips Eigen solver instead of bundle adjustment (BA) failed!");
                if (mRefineBA > 0)
                {
                    ROS_WARN("Trying bundle adjustment instead!");
                    poselib::triangPts3D(R, t, p0, p1, Q, mask, mMaxDist3DPtsZ);
                }
                else
                {
                    ROS_WARN("Trying refinement with weighted (Pseudo-Huber) Stewenius instead!");
                    int refineMethod_tmp = poselib::RefinePostAlg::PR_STEWENIUS | poselib::RefinePostAlg::PR_PSEUDOHUBER_WEIGHTS;;
                    if (!poselib::refineEssentialLinear(p0, p1, E, mask, refineMethod_tmp, nr_inliers, cv::noArray(), cv::noArray(), th, 4, 2.0, 0.1, 0.15))
                    {
                        Q.release();
                        poselib::getPoseTriangPts(E, p0, p1, R, t, Q, mask, mMaxDist3DPtsZ);
                    }
                    else
                        ROS_WARN("Refinement failed!");
                }
            }
        }

        if (useBA)
        {
            if (mRefineBA == 1)
            {
              //ROS_INFO("refine stereo ba");
              poselib::refineStereoBA(p0, p1, R, t, Q, mK0, mK1, false, mask);
              //ROS_INFO("end refine");
            }
            else if (mRefineBA == 2)
            {
              poselib::CamToImgCoordTrans(p0, mK0);
              poselib::CamToImgCoordTrans(p1, mK1);
              poselib::refineStereoBA(p0, p1, R, t, Q, mK0, mK1, true, mask);
            }
        }

        if (4 < mVerbosityLevel)
        {
          time = 1000 * ((double) cv::getTickCount() - time) / cv::getTickFrequency();
          time_total +=time;

          ROS_INFO("Time for bundle adjustment: %2.2f ms", time);
        }

        if (5 < mVerbosityLevel)
        {
          ROS_INFO("Overall pose estimation time: %2.2f ms", time_total);
          ROS_INFO("Number of inliers: %d", cv::countNonZero(mask));
        }

        //double roll, pitch, yaw;
        //poselib::getAnglesRotMat(R, roll, pitch, yaw);
        //std::cout << "Angles of estimated rotation: roll = " << std::setprecision(4) << roll << char(248) << " pitch = " << pitch << char(248) << " yaw = " << yaw << char(248) << std::endl;
        //std::cout << "Estimated translation vector: [ " << std::setprecision(4) << t.at<double>(0) << " " << t.at<double>(1) << " " << t.at<double>(2) << " ]" << std::endl;

        publish_confidence(confidence);
    }
    else
    {
        if ((evStepStereoStable_cnt == evStepStereoStable_tmp) || (evStepStereoStable_cnt == 0))
        {
            if(!stereoObj)
            {
                ROS_ERROR("No stereo refinement possible as the corresponding object was not created in the SW!");
                return;
            }
            if (stereoObj->addNewCorrespondences(finalMatches, kp0, kp1, mUsacConfig) != -1)
            {
                R = stereoObj->R_new;
                t = stereoObj->t_new;
            }
            else
            {
                ROS_WARN("Pose estimation failed!");
                return;
            }

            if(evStepStereoStable_cnt == 0)
                evStepStereoStable_cnt = evStepStereoStable_tmp;

            if (mbUseMostLikelyPose && stereoObj->mostLikelyPose_stable)
            {
                R = stereoObj->R_mostLikely;
                t = stereoObj->t_mostLikely;
            }

            if (stereoObj->poseIsStable)
            {
                evStepStereoStable_cnt--;
                poseWasStable = true;
                R.copyTo(R_stable);
                t.copyTo(t_stable);
            }
            else if (poseWasStable && mbUseOnlyStablePose && !(mbUseMostLikelyPose && stereoObj->mostLikelyPose_stable))
            {
                R_stable.copyTo(R);
                t_stable.copyTo(t);
            }
        }
        else
        {
            evStepStereoStable_cnt--;
            R = R_stable;
            t = t_stable;
        }
    }
  }
  else
  {
    // no pose estimation - set R & t
    R = mR1;
    t = mt1;
  }

  //get translation vector for mapping left (camera) coordinates to right (camera) coordinates
//  R0 = R.t();
//  t0 = -1.0 * R.t() * t;
  R0 = R;
  t0 = t;

  publish_pose(R0, t0);

  //R = cv::Mat::eye(3,3,CV_64FC1);

  cv::Mat rect0, rect1, K0, K1, mapX0, mapY0, mapX1, mapY1;

  // get rectification matrices
  poselib::getRectificationParameters(R, t, mK0, mK1, mDist0_8, mDist1_8, cv::Size(img0.cols, img0.rows), rect0, rect1, K0, K1, mRectAlpha);

  //get rectification maps
  cv::initUndistortRectifyMap(mK0, mDist0_8, rect0, K0, cv::Size(img0.cols, img0.rows), CV_32FC1, mapX0, mapY0);
  cv::initUndistortRectifyMap(mK1, mDist1_8, rect1, K1, cv::Size(img0.cols, img0.rows), CV_32FC1, mapX1, mapY1);

  cv::Mat outImg0, outImg1;
  //poselib::ShowRectifiedImages(img0, img1, mapX0, mapY0, mapX1, mapY1, t0, "", cv::Size(img0.cols, img0.rows));
  poselib::GetRectifiedImages(img0, img1, mapX0, mapY0, mapX1, mapY1, t0, outImg0, outImg1);

  //cv::imshow("img0", outImg0);

  //ROS_INFO("rect info 0: %d x %d - %d", (int) outImg0.cols, (int) outImg0.rows, (int) outImg0.step);
  //ROS_INFO("rect info 1: %d x %d - %d", (int) outImg1.cols, (int) outImg1.rows, (int) outImg1.step);
  publish_rectified_images(outImg0, outImg1);
  publish_cam_info(K0, mDist0_5, rect0, K1, mDist1_5, rect1);

  std::cout << std::endl << std::endl;
}

void ROSnode::publish_confidence(double& conf)
{
  std_msgs::Float32 msg;
  msg.data = conf;

  mPubConfidence.publish(msg);
}

void ROSnode::publish_rectified_images(cv::Mat& imgLt, cv::Mat& imgRt)
{

  mImgLt.encoding = "mono8";
  mImgRt.encoding = "mono8";
  mImgLt.image = imgLt;
  mImgRt.image = imgRt;

  sensor_msgs::ImagePtr msgLt = mImgLt.toImageMsg();
  sensor_msgs::ImagePtr msgRt = mImgRt.toImageMsg();

  msgLt.get()->header.stamp = mTime;
  msgRt.get()->header.stamp = mTime;

  mPubImgLt.publish(msgLt);
  mPubImgRt.publish(msgRt);
}

void ROSnode::publish_pose (cv::Mat& R, cv::Mat& t)
{
  geometry_msgs::PoseStamped pose;

  pose.header.stamp = mTime;

  //std::cout << "t: [ x y z ]" << std::endl;
  //std::cout << "t: [ " << std::setprecision(4) << t.at<double>(0) << " " << t.at<double>(1) << " " << t.at<double>(2) << " ]" << std::endl;
  pose.pose.position.x = t.at<double>(0);
  pose.pose.position.y = t.at<double>(1);
  pose.pose.position.z = t.at<double>(2);

  double roll, pitch, yaw;
  poselib::getAnglesRotMat(R, roll, pitch, yaw);
  std::cout << "Angles of estimated rotation: roll = " << std::setprecision(4) << roll << " pitch = " << pitch << " yaw = " << yaw << std::endl;

  // mat to q
  Eigen::Matrix3d eigR;
  Eigen::Vector4d q;
  cv::cv2eigen(R, eigR);
  poselib::MatToQuat(eigR, q);

  //std::cout << "q: [ w x y z ]" << std::endl;
  //std::cout << "q: [ " << std::setprecision(4) << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " ]" << std::endl;
  pose.pose.orientation.w = q(0);
  pose.pose.orientation.x = q(1);
  pose.pose.orientation.y = q(2);
  pose.pose.orientation.z = q(3);

  mPubPose.publish(pose);

  // publish pose diff
  if (mbInputPose)
  {
    // calculate R, t diff single value
    double valRdiff, valTdiff;
    cv::Mat t1 = mt1;
    if(!poselib::nearZero(1000*cv::norm(mt1)))
    {
        t1 = mt1 / cv::norm(mt1);
    }
    poselib::compareRTs(R, mR1, t, t1, &valRdiff, &valTdiff, false);

    // create ros pose - calculate difference
    cv::Mat Rdiff = mR1.t() * R;
    Eigen::Matrix3d eigRdiff;
    cv::cv2eigen(Rdiff, eigRdiff);
    poselib::MatToQuat(eigRdiff, q);

    cv::Mat tdiff = t - t1;
    pose.pose.position.x = tdiff.at<double>(0);
    pose.pose.position.y = tdiff.at<double>(1);
    pose.pose.position.z = tdiff.at<double>(2);

    pose.pose.orientation.w = q(0);
    pose.pose.orientation.x = q(1);
    pose.pose.orientation.y = q(2);
    pose.pose.orientation.z = q(3);

    mPubPoseDiff.publish(pose);
  }
}

void ROSnode::publish_cam_info (cv::Mat& K0, cv::Mat& d0, cv::Mat& rect0, cv::Mat& K1, cv::Mat& d1, cv::Mat& rect1)
{
  // create camera info
  sensor_msgs::CameraInfo ci0, ci1;

  ci0.distortion_model = "plumb_bob";
  ci1.distortion_model = "plumb_bob";

  //int maxH = 1;
  //int maxW = 5;

  ci0.D = {d0.at<double>(0, 1), d0.at<double>(0, 2), d0.at<double>(0, 3), d0.at<double>(0, 4), d0.at<double>(0, 5)};
  ci1.D = {d1.at<double>(0, 1), d1.at<double>(0, 2), d1.at<double>(0, 3), d1.at<double>(0, 4), d1.at<double>(0, 5)};

  //maxH = 3;
  //maxW = 3;
  ci0.K = {K0.at<double>(0, 0), K0.at<double>(0, 1), K0.at<double>(0, 2), K0.at<double>(1, 0), K0.at<double>(1, 1), K0.at<double>(1, 2), K0.at<double>(2, 0), K0.at<double>(2, 1), K0.at<double>(2, 2)};
  ci1.K = {K1.at<double>(0, 0), K1.at<double>(0, 1), K1.at<double>(0, 2), K1.at<double>(1, 0), K1.at<double>(1, 1), K1.at<double>(1, 2), K1.at<double>(2, 0), K1.at<double>(2, 1), K1.at<double>(2, 2)};
  ci0.R = {rect0.at<double>(0, 0), rect0.at<double>(0, 1), rect0.at<double>(0, 2), rect0.at<double>(1, 0), rect0.at<double>(1, 1), rect0.at<double>(1, 2), rect0.at<double>(2, 0), rect0.at<double>(2, 1), rect0.at<double>(2, 2)};
  ci1.R = {rect1.at<double>(0, 0), rect1.at<double>(0, 1), rect1.at<double>(0, 2), rect1.at<double>(1, 0), rect1.at<double>(1, 1), rect1.at<double>(1, 2), rect1.at<double>(2, 0), rect1.at<double>(2, 1), rect1.at<double>(2, 2)};

  ci0.header.stamp = mTime;
  ci1.header.stamp = mTime;


  mPubCamLt.publish(ci0);
  mPubCamRt.publish(ci1);
}

void ROSnode::publish_matches (cv::Mat& img)
{
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

  msg->header.stamp = mTime;

  mPubMatches.publish(msg);
}

void ROSnode::callback_mono(const sensor_msgs::ImageConstPtr &_img)
{
  std::shared_ptr<BufferData> img = std::make_shared<BufferData>(_img->header.stamp, cv_bridge::toCvCopy(_img, "mono8"));
  mBuffer.push_back(img);
  mbHasNewMessage = true;
}

void ROSnode::callback_mono_pose(const sensor_msgs::ImageConstPtr &_img, const geometry_msgs::PoseStampedConstPtr &_pose)
{
  std::shared_ptr<BufferData> imgPose = std::make_shared<BufferData>(_img->header.stamp, cv_bridge::toCvCopy(_img, "mono8"), _pose);
  mBuffer.push_back(imgPose);
  mbHasNewMessage = true;
}

void ROSnode::callback_stereo(const sensor_msgs::ImageConstPtr &_imgLt, const sensor_msgs::ImageConstPtr &_imgRt)
{
  std::shared_ptr<BufferData> imgStereo = std::make_shared<BufferData>(_imgLt->header.stamp, cv_bridge::toCvCopy(_imgLt, "mono8"), cv_bridge::toCvCopy(_imgRt, "mono8"));
  mBuffer.push_back(imgStereo);
  mbHasNewMessage = true;
}

void ROSnode::callback_stereo_pose(const sensor_msgs::ImageConstPtr &_imgLt, const geometry_msgs::PoseStampedConstPtr &_poseLt, const sensor_msgs::ImageConstPtr &_imgRt, const geometry_msgs::PoseStampedConstPtr &_poseRt)
{
  std::shared_ptr<BufferData> imgStereoPose = std::make_shared<BufferData>(_imgLt->header.stamp, cv_bridge::toCvCopy(_imgLt, "mono8"), _poseLt, cv_bridge::toCvCopy(_imgRt, "mono8"), _poseRt);
  mBuffer.push_back(imgStereoPose);
  mbHasNewMessage = true;
}

void ROSnode::callback_cam_info_mono(const sensor_msgs::CameraInfoConstPtr &_cam)
{
  if (!mbHasCameraInfo)
  {
    if (_cam->distortion_model != "plumb_bob")
    {
      ROS_WARN("CameraInfo - Distortion Model not usable.");
      return;
    }
    // set K0, K1, dist
    int maxW = 3;
    int maxH = 3;
    mK0 = cv::Mat(cv::Size(maxW,maxH), CV_64FC1);
    for (int y = 0; y < maxH; ++y)
    {
      for (int x = 0; x < maxW; ++x)
      {
        mK0.at<double>(y, x) = _cam->K.at((y*maxH)+x);
      }
    }

    mK1 = mK0;

    maxW = 5;
    maxH = 1;
    mDist0_5 = cv::Mat(cv::Size(maxW, maxH), CV_64FC1);
    for (int y = 0; y < maxH; ++y)
    {
      for (int x = 0; x < maxW; ++x)
      {
        mDist0_5.at<double>(y, x) = _cam->D.at((y*maxH)+x);
      }
    }

    mDist0_8 = cv::Mat::zeros(1,8, mDist0_5.type());
    if (mDist0_5.rows > mDist0_5.cols)
    {
      mDist0_5 = mDist0_5.t();
    }
    mDist0_5.copyTo(mDist0_8.colRange(0,5));

    mDist1_5 = mDist0_5;
    mDist1_8 = mDist0_8;

    mPxToCamFact = 4. / (std::sqrt(2.) * (mK0.at<double>(1,1) + mK0.at<double>(2,2) + mK1.at<double>(1,1) + mK1.at<double>(2,2)));

    mbHasCameraInfo = true;

  }
}

void ROSnode::callback_cam_info_stereo(const sensor_msgs::CameraInfoConstPtr &_camLt, const sensor_msgs::CameraInfoConstPtr &_camRt)
{
  if (!mbHasCameraInfo)
  {
    if (_camLt->distortion_model != "plumb_bob" || _camRt->distortion_model != "plumb_bob")
    {
      ROS_WARN("CameraInfo - Distortion Model not usable.");
      return;
    }
    // set K0, K1, dist
    int maxW = 3;
    int maxH = 3;
    mK0 = cv::Mat(cv::Size(maxW,maxH), CV_64FC1);
    mK1 = cv::Mat(cv::Size(maxW,maxH), CV_64FC1);
    for (int y = 0; y < maxH; ++y)
    {
      for (int x = 0; x < maxW; ++x)
      {
        mK0.at<double>(y, x) = _camLt->K.at((y*maxH)+x);
        mK1.at<double>(y, x) = _camRt->K.at((y*maxH)+x);
      }
    }

    maxW = 5;
    maxH = 1;
    mDist0_5 = cv::Mat(cv::Size(maxW, maxH), CV_64FC1);
    mDist1_5 = cv::Mat(cv::Size(maxW, maxH), CV_64FC1);
    for (int y = 0; y < maxH; ++y)
    {
      for (int x = 0; x < maxW; ++x)
      {
        mDist0_5.at<double>(y, x) = _camLt->D.at((y*maxH)+x);
        mDist1_5.at<double>(y, x) = _camRt->D.at((y*maxH)+x);
      }
    }

    mDist0_8 = cv::Mat::zeros(1,8, mDist0_5.type());
    if (mDist0_5.rows > mDist0_5.cols)
    {
      mDist0_5 = mDist0_5.t();
    }
    mDist0_5.copyTo(mDist0_8.colRange(0,5));

    mDist1_8 = cv::Mat::zeros(1,8, mDist1_5.type());
    if (mDist1_5.rows > mDist1_5.cols)
    {
      mDist1_5 = mDist1_5.t();
    }
    mDist1_5.copyTo(mDist1_8.colRange(0,5));

    mPxToCamFact = 4. / (std::sqrt(2.) * (mK0.at<double>(1,1) + mK0.at<double>(2,2) + mK1.at<double>(1,1) + mK1.at<double>(2,2)));

    //For stereo refinement
    if(mbStereoRef)
    {
        cfg_stereo.dist0_8 = &mDist0_8;
        cfg_stereo.dist1_8 = &mDist1_8;
        cfg_stereo.K0 = &mK0;
        cfg_stereo.K1 = &mK1;

        if(stereoObj)
        {
            stereoObj->setNewParameters(cfg_stereo);
        }
        else
        {
            stereoObj.reset(new poselib::StereoRefine(cfg_stereo));
        }
    }

    mbHasCameraInfo = true;
  }
}

void ROSnode::draw_matches(cv::Mat& img1, cv::Mat& img2, std::vector<cv::KeyPoint> kp1, std::vector<cv::KeyPoint> kp2, std::vector<cv::DMatch> matches, cv::Mat& outImg)
{
  if (mNumDrawnMatches <= 0)
  {
    if (mNumDrawnMatches == -2)
    {
      cv::drawMatches(img1, kp1, img2, kp2, matches, outImg);
    }
    else
    {
      cv::drawMatches(img1, kp1, img2, kp2, matches, outImg, cv::Scalar::all(-1), cv::Scalar(43, 112, 175), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    }
  }
  else
  {
    //Show reduced set of matches
    std::vector<cv::KeyPoint> keypL_reduced;//Left keypoints
    std::vector<cv::KeyPoint> keypR_reduced;//Right keypoints
    std::vector<cv::DMatch> matches_reduced;
    std::vector<cv::KeyPoint> keypL_reduced1;//Left keypoints
    std::vector<cv::KeyPoint> keypR_reduced1;//Right keypoints
    std::vector<cv::DMatch> matches_reduced1;
    size_t keepNMatches = mNumDrawnMatches;
    if(matches.size() > keepNMatches)
    {
      size_t keepXthMatch = matches.size() / keepNMatches;

      for (unsigned int i = 0; i < matches.size(); ++i)
      {
        int idx = matches[i].queryIdx;
        keypL_reduced.push_back(kp1[idx]);
        matches_reduced.push_back(matches[i]);
        matches_reduced.back().queryIdx = i;
        keypR_reduced.push_back(kp2[matches_reduced.back().trainIdx]);
        matches_reduced.back().trainIdx = i;
      }

      for (unsigned int i = 0, j = 0; i < matches_reduced.size(); ++i)
      {
        if((i % (int)keepXthMatch) == 0)
        {
          keypL_reduced1.push_back(keypL_reduced[i]);
          matches_reduced1.push_back(matches_reduced[i]);
          matches_reduced1.back().queryIdx = j;
          keypR_reduced1.push_back(keypR_reduced[i]);
          matches_reduced1.back().trainIdx = j;
          j++;
        }
      }

      drawMatches(img1, keypL_reduced1, img2, keypR_reduced1, matches_reduced1, outImg);
    }
  }
}

bool ROSnode::select_images_mono(cv::Mat& img0, cv::Mat& R0, cv::Mat& t0, cv::Mat& img1, cv::Mat& R1, cv::Mat& t1)
{
  if (mBuffer.size() < 2)
  {
    ROS_INFO("not enough elements available\n");
    return false;
  }

  std::shared_ptr<BufferData> data0 = *boost::prior(mBuffer.end(), 1); // get last element
  std::shared_ptr<BufferData> data1;

  switch (mSelectionType)
  {
    case k_all:
    {
      data1 = *boost::prior(mBuffer.end(), 2); // get previous to last element - mBuffer.at(mBuffer.size()-2);
    } break;
    case k_skip:
    {
      if ((int)mBuffer.size() < mSelectionSkipCount + 2)
      {
        return false;
      }
      data1 = *boost::prior(mBuffer.end(), 1 + mSelectionSkipCount + 1); // mBuffer.at(mBuffer.size() - 1 - mSelectionSkipCount - 1);
    } break;
    case k_time:
    {
      // reverse loop through buffer, find first image with appropriate time difference
      bool found = false;
      for (size_t i = 1; i < mBuffer.size(); ++i) // start at previous to last element
      {
        data1 = *boost::prior(mBuffer.end(), i + 1);
        if (data1->GetTimeNS() <= data0->GetTimeNS() - mSelectionTimeMS*1000000) // time diff found
        {
          found = true;
          break;
        }
      }
      if (!found)
      {
        return false;
      }
    } break;
    case k_baseline:
    {
      // reverse loop through buffer, find first image with appropriate baseline
      bool found = false;
      double baseline = 0;
      for (size_t i = 1; i < mBuffer.size(); ++i) // start at previous to last element
      {
        data1 = *boost::prior(mBuffer.end(), i + 1);
        if (!data0->GetBaseline(baseline, data1))
        {
          return false;
        }
        else if (baseline >= mSelectionBaseline)
        {
          found = true;
          break;
        }
      }
      if (!found)
      {
        return false;
      }
    } break;
  }

  // data0 and data1 are filled
  mTime = data1->GetRosTime();

  data0->GetImage(img1);
  data1->GetImage(img0);

  data1->GetRI(R0); // R0 = I
  data1->Gett0(t0); // t0 = 0

  if (mbInputPose)
  {
    data1->GetRelPose(R1, t1, data0); // set R1 and t1
  }
  else
  { // no pose given
    R1 = R0;
    t1 = t0;
  }

  return true;
}

bool ROSnode::select_images_stereo(cv::Mat& img0, cv::Mat& R0, cv::Mat& t0, cv::Mat& img1, cv::Mat& R1, cv::Mat& t1)
{
  std::shared_ptr<BufferData> data;

  if (mBuffer.size() > 1)
  {
    switch (mSelectionType)
    {
      case k_all:
      case k_baseline: // baseline cannot be varied in stereo mode -> take all
      {
        data = *boost::prior(mBuffer.end(), 1); // take last element
      } break;
      case k_skip:
      {
        std::shared_ptr<BufferData> prev;
        if (!(prev = mPrevStereo.lock()))
        {
          // previous element is not in buffer anymore -> take last element
          data = *boost::prior(mBuffer.end(), 1);
        }
        else if ((int)mBuffer.size() < mSelectionSkipCount + 2)
        {
          // not enough elements available
          return false;
        }
        else {
          // element is still in buffer, check if enough entries were inserted
          data = *boost::prior(mBuffer.end(), 1 + mSelectionSkipCount + 1); // (1 = last, skipCount = skip # entries, 1 = take previous)
          if (data.get() != prev.get()) // previous element is not at required position
          {
              if(data->GetTimeNS() > prev->GetTimeNS())
              {
                  //More elements then expected have arrived since the last call as the process for pose estimation was too slow. Taking last element.
                  data = *boost::prior(mBuffer.end(), 1); // take last element
              }
              else
              {
                  return false;
              }
          }
        }
      } break;
      case k_time:
      {
        std::shared_ptr<BufferData> prev;
        if (!(prev = mPrevStereo.lock()))
        {
          // previous element is not in buffer anymore -> take last element
          data = *boost::prior(mBuffer.end(), 1);
        }
        else
        {
          // prev element is still in buffer -> check if current element fits time difference
          data = *boost::prior(mBuffer.end(), 1);
          if (prev->GetTimeNS() > data->GetTimeNS() - mSelectionTimeMS*1000000)
          {
            return false; // time diff doesn't fit
          }
        }
      } break;
    }
  }
  else if (mBuffer.size() == 1) // only one entry available
  {
    // take first entry - should be first run of motion_rectification_ros_node
    data = mBuffer.at(0);
  }

  mPrevStereo = data;

  // data is filled
  mTime = data->GetRosTime();

  data->GetImage(img0);
  data->GetImage1(img1);

  data->GetRI(R0); // R0 = I
  data->Gett0(t0); // t0 = 0

  if (!data->GetRelPose(R1, t1))
  {
    // no relative pose available
    R1 = R0;
    t1 = t0;
  }

  return true;
}

void ROSnode::_param_get_sizet(size_t &variable, const std::string &param, const size_t &default_value)
{
  XmlRpc::XmlRpcValue xmlValue;
  mNh.getParam(param, xmlValue);

  if(xmlValue.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_WARN("Parameter %s invalid or not found! Set to default value %d", param.c_str(), (int)default_value);
    variable = default_value;
  }
  else
  {
    variable = (size_t)static_cast<int>(xmlValue);
    ROS_INFO("%s value loaded from config file: %d", param.c_str(), (int)variable);
  }
}
