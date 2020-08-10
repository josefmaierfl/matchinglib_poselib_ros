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
