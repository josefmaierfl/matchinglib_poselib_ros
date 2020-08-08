/**
 * Copyright (C) 2016 by Austrian Institute of Technology
 *
 * @file  InitSubscriber.hpp
 *
 * @brief  subscribs a topic and wait for a single message.
 *         refactore init_subscriber.h
 *
 * @author  Roland Jung (Roland.Jung@ait.ac.at)
 *
 * @date  14.12.2016
 */
#ifndef INITSUBSCRIBER_HPP
#define INITSUBSCRIBER_HPP

#include <functional>
#include <string>
#include <ros/ros.h>

namespace ait_common
{
  template<typename T>
  class InitSubscriber
  {

  private:
    std::string     mTopic;
    ros::NodeHandle mNh;
    ros::Subscriber mSub;
    bool            mReceived;
  public:
    T data;
    boost::shared_ptr< T const> ptrData;

    InitSubscriber(ros::NodeHandle &nh) : mReceived(false), mNh(nh)
    { }

    InitSubscriber(ros::NodeHandle &nh, std::string const &topic) : mReceived(false), mNh(nh), mTopic(topic)
    {
      subscribe();
    }

    void callback(boost::shared_ptr< T const> msg)
    {
      ptrData = msg;
      data = *msg;
      mSub.shutdown();
      mReceived = true;
    }

    void subscribe(std::string const &topic = "")
    {
      if(!topic.empty())
      {
        mTopic = topic;
      }
      mReceived = false;
      mSub      = mNh.subscribe<T>(mTopic, 1, boost::bind(&InitSubscriber<T>::callback, this, _1));
      ROS_INFO("InitSubscriber: subscribed topic %s", mTopic.c_str());
    }

    void waitForMessage()
    {
      ROS_INFO("Waiting for message: %s", mTopic.c_str());
      while (!mReceived)
      {
        usleep(100000);
        ros::spinOnce();
      }
      ROS_INFO(" ... received message");
    }

  };

} // namespace ait_common

#endif // INITSUBSCRIBER_HPP
