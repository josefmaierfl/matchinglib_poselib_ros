/**
 * Copyright (C) 2016 by Austrian Institute of Technology
 *
 * @file  Node.hpp
 *
 * @brief  refactored pfe_node.h
 *
 * @author  Roland Jung (Roland.Jung@ait.ac.at)
 *
 * @date  14.12.2016
 */
#ifndef AIT_COMMON_NODE_HPP
#define AIT_COMMON_NODE_HPP
#include <cassert>
#include <ros/ros.h>
#include <cstdint>

using std::uint32_t;
using std::uint16_t;
using std::int16_t;

namespace ait_common
{

  /**
   * abstract baseclass for node with custom spinning
   * order of execution of virtual methods:
   *  init_topics() subscribe and publish topics, initialize message independet stuff
   *
   *  init() called after first message is arrived
   *  process() called with each message
   *
   *  signal run() that new data is arrived with the mbHasNewMessage flag
   *
   *  params:
   *   node_profile: enable timing mesauremnet for process() and overhead
   *   node_spinning_rate: desired spinning rate [Hz]. default 50 Hz
   *   node_warn_time: warn if no message arrived in X s. default 15 (0 do disable)
   *
   */
  class Node
  {
    protected:
      Node(ros::NodeHandle &nh) : mNh(nh), mbIsInitialized(false), mbHasNewMessage(false)
      { }

    public:
      void run();

      void setProfiling(const bool val);
      bool isInitialized();

    protected:
      /**
       * subscribe and publish topics, initialize message independet stuff
       */
      virtual void init_topics() = 0;

      /**
         * initialize node; called after first message received
         */
      virtual void init() = 0;

      /**
         * process message. called when mbHasNewMessage was set
         */
      virtual void process() = 0;


      void _get_param_u32(std::string const &key, std::uint32_t &value, std::uint32_t const &def);

      void _get_param_s16(std::string const &key, std::int16_t &value, std::int16_t const &def);

      void _param_get_string(std::string &variable, std::string const &param, std::string const &default_value);

      void _param_get_int(int &variable, std::string const &param, int const &default_value);

      void _param_get_uint(unsigned &variable, std::string const &param, unsigned const &default_value);

      void _param_get_float(float &variable, std::string const &param, float const &default_value);

      template<typename T>
      void _param_check_range(T &variable, std::string const &param_name, T const &min, T const &max,
                              T const &default_value)
      {
        if(variable < min || variable > max)
        {
          variable = default_value;
          ROS_WARN_STREAM(param_name << " invalid value (" << variable << ")! Value set to " << default_value);
        }
      }

      /// MEMBERS:

      ros::NodeHandle mNh;

      bool mbIsInitialized;

      bool mbHasNewMessage;

      float mPeriod_s;

      bool mbProfile = true;
    private:
      Node();

      Node(const Node &);

      Node &operator=(const Node &);

      /**
       * Print all subscritpions and advertations
       */
      void printTopicInfo();

  };

} // namespace ait_common

#endif // AIT_COMMON_ROSNODE_HPP
