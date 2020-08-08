#include "../include/ait_common/Node.hpp"


void ait_common::Node::run()
{
  init_topics();
  printTopicInfo();

  // profile process() function and overheader
  mNh.param<bool>("node_profile", mbProfile, true);
  ros::Time processTimeStop;
  ros::Time processTimeStart;

  // callback spinning rate
  int spinningRate; // Hz
  mNh.param<int>("node_spinning_rate", spinningRate, 50);

  // warning message
  int warnAfter_s; //s
  mNh.param<int>("node_warn_time", warnAfter_s, 15);


  unsigned int emptySpinCnt     = 0;
  unsigned int  warningEmptySpin = spinningRate * warnAfter_s;

  if(warningEmptySpin == 0)
  {
    warningEmptySpin = std::numeric_limits<unsigned>::max();
  }

  ros::Rate    rate(spinningRate);

  while (ros::ok())
  {
    ros::spinOnce();

    if(mbHasNewMessage)
    {
      if(!mbIsInitialized)
      {
        ROS_INFO("Initialize");
        init();
        ROS_INFO("Initialization done!");
        mbIsInitialized = true;
      }

      processTimeStart = ros::Time::now();
      ros::Duration overheadTime = processTimeStart - processTimeStop;
      process();
      processTimeStop = ros::Time::now();
      ros::Duration processTime = processTimeStop - processTimeStart;
      mPeriod_s    = (processTime + overheadTime).toSec();

      if(mbProfile)
      {

        double        process_hz  = (processTime.toSec() > 0) ? 1 / processTime.toSec() : 0;
        double        period_hz   = (mPeriod_s) ? 1 / mPeriod_s : 0;
        ROS_INFO("process() %fs (%f Hz), overhead %fs, (%f Hz)", processTime.toSec(), process_hz,
                 overheadTime.toSec(), period_hz);
      }

      mbHasNewMessage = false; // message is processed
      emptySpinCnt   = 0;
    }
    else
    {
      emptySpinCnt++;

      if(emptySpinCnt >= warningEmptySpin)
      {
        ROS_WARN("No message received in the last %d s", warnAfter_s);
        emptySpinCnt = 0;
      }
    }

    rate.sleep();
  }
}

void ait_common::Node::setProfiling(const bool val)
{
  mbProfile = val;
}

bool ait_common::Node::isInitialized()
{
  return mbIsInitialized;
}

void ait_common::Node::_get_param_u32(const std::string &key, std::uint32_t &value, const std::uint32_t &def)
{
  int _val;
  mNh.param<int>(key, _val, def);
  value = static_cast<std::uint32_t>(_val);
}

void ait_common::Node::_get_param_s16(const std::string &key, std::int16_t &value, const std::int16_t &def)
{
  int _val;
  mNh.param<int>(key, _val, def);
  value = static_cast<std::int16_t>(_val);
}

void ait_common::Node::_param_get_string(std::string &variable, const std::string &param, const std::string &default_value)
{
  XmlRpc::XmlRpcValue xmlValue;
  mNh.getParam(param, xmlValue);

  if(xmlValue.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    ROS_WARN("Parameter %s invalid or not found! Set to default value %s", param.c_str(),
             default_value.c_str());
    variable = default_value;
  }
  else
  {
    variable = static_cast<std::string>(xmlValue);
    ROS_INFO("%s value loaded from config file: %s", param.c_str(), variable.c_str());
  }
}

void ait_common::Node::_param_get_int(int &variable, const std::string &param, const int &default_value)
{
  XmlRpc::XmlRpcValue xmlValue;
  mNh.getParam(param, xmlValue);

  if(xmlValue.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_WARN("Parameter %s invalid or not found! Set to default value %d", param.c_str(), default_value);
    variable = default_value;
  }
  else
  {
    variable = static_cast<int>(xmlValue);
    ROS_INFO("%s value loaded from config file: %d", param.c_str(), variable);
  }
}

void ait_common::Node::_param_get_uint(unsigned &variable, const std::string &param, const unsigned &default_value)
{
  XmlRpc::XmlRpcValue xmlValue;
  mNh.getParam(param, xmlValue);

  if(xmlValue.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_WARN("Parameter %s invalid or not found! Set to default value %d", param.c_str(), default_value);
    variable = default_value;
  }
  else
  {
    variable = (unsigned)static_cast<int>(xmlValue);
    ROS_INFO("%s value loaded from config file: %d", param.c_str(), variable);
  }
}

void ait_common::Node::_param_get_float(float &variable, const std::string &param, const float &default_value)
{
  XmlRpc::XmlRpcValue xmlValue;
  mNh.getParam(param, xmlValue);

  if(xmlValue.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    ROS_WARN("Parameter %s invalid or not found! Set to default value %f", param.c_str(), default_value);
    variable = default_value;
  }
  else
  {
    variable = (float) static_cast<double>(xmlValue);
    ROS_INFO("%s value loaded from config file: %f", param.c_str(), variable);
  }
}

void ait_common::Node::printTopicInfo()
{
  //  print published/subscribed topics
  std::string   nodeName = ros::this_node::getName();
  ros::V_string topics;
  ros::this_node::getSubscribedTopics(topics);
  std::string       topicsStr = nodeName + ":\n\tsubscribed to topics:\n";

  for (unsigned int i         = 0; i < topics.size(); i++)
  {
    topicsStr += ("\t\t" + topics.at(i) + "\n");
  }

  topicsStr += "\tadvertised topics:\n";
  ros::this_node::getAdvertisedTopics(topics);

  for (unsigned int i = 0; i < topics.size(); i++)
  {
    topicsStr += ("\t\t" + topics.at(i) + "\n");
  }

  ROS_INFO_STREAM("" << topicsStr);
}
