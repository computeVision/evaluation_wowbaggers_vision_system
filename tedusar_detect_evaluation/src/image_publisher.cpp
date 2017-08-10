/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Peter Lorenz
 *                      Institute for Software Technology,
 *                      Graz University of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Graz University of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <tedusar_detect_evaluation/image_publisher.h>
#include "tedusar_detect_evaluation/my_exception.h"
#include <tedusar_detect_evaluation/RectImage.h>
#include <image_transport/image_transport.h>
#include <tedusar_feature_msgs/Feature.h>
#include <tedusar_feature_msgs/SensorOutput.h>
#include <sensor_msgs/image_encodings.h>

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <dirent.h>

namespace image_publisher
{
  using namespace ros;
  using namespace std;

  ImagePublisher::ImagePublisher() : nh_("~")
  {
    nh_.getParam("dir", dir_);
    nh_.getParam("debug", debug_);
    nh_.getParam("subscribe", subscribe_path_);
    nh_.getParam("cnn", cnn_);
    nh_.getParam("pub_rect", pub_rect_path_);
    nh_.getParam("pub_roc", pub_roc_path_);
    nh_.getParam("pub_cnn", pub_cnn_path_);

    msg_rect_.subscribed = 0;

    pub_rect_ = nh_.advertise<tedusar_detect_evaluation::RectImage>(pub_rect_path_, 1);
    pub_roc_ = nh_.advertise<tedusar_detect_evaluation::RectImage>(pub_roc_path_, 4);
    pub_cnn_ = nh_.advertise<sensor_msgs::Image>(pub_cnn_path_, 1);

    cout << "[Image_Publisher] Publish " << endl;
    if(cnn_)
      sub_ = nh_.subscribe(subscribe_path_, 1, &ImagePublisher::imageCallCnn, this);
    else
      sub_ = nh_.subscribe(subscribe_path_, 4, &ImagePublisher::imageCall, this);
  }

  ImagePublisher::~ImagePublisher()
  {

  }

  void ImagePublisher::imageCallCnn(const tedusar_feature_msgs::SensorOutputPtr &msg)
  {
    if (debug_)
    {
      cout << "[imageCallCnn] " << msg_rect_.image_name << endl;
      cout << "[imageCallCnn] features " << msg->detectors.data()->features.empty() << endl;
      cout << "[imageCallCnn] probabil " << msg->detectors.data()->probability_seen << endl;
      cout << "[imageCallCnn] features " << msg->detectors.data()->feature_type.empty() << endl;
    }

    if(msg->detectors.data()->features.empty())
      msg_rect_.cnn = 0;
    else
      msg_rect_.cnn = 1;

    cout << "[imageCallCnn] " << msg_rect_.cnn << endl;

    pub_roc_.publish(msg_rect_);
  }

  void ImagePublisher::imageCall(const tedusar_detect_evaluation::RectImagePtr &msg)
  {
    cout << "[Image_Publisher] image Call haarcascade " << endl;
    pub_roc_.publish(msg);
  }

  void ImagePublisher::init()
  {
    std::vector<std::string> tmp;
    readDir(tmp, dir_);
    ROS_WARN_STREAM(to_string(tmp.size()));

    cout << "Waiting for Subscriber... " << endl;
    ros::Rate poll_rate(40);
    while(pub_cnn_.getNumSubscribers() == 0 and pub_rect_.getNumSubscribers() == 0)
      poll_rate.sleep();
    cout << "... Subscriber Connected." << endl;

    for(auto imag : tmp)
    {
      if(not ros::ok())
        break;

      cv::Mat image = cv::imread(dir_ + imag, CV_LOAD_IMAGE_COLOR);
      ROS_WARN_STREAM("[init_pub] " + imag);

      //publish images
      msg_rect_.image_name = imag;
      msg_rect_.dir = dir_;

      if(cnn_)
      {
        sensor_msgs::Image::Ptr small_image = boost::make_shared<sensor_msgs::Image>();
        msg_rect_.sensor_msg = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        small_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        pub_cnn_.publish(small_image);
      }
      else
      {
        msg_rect_.sensor_msg = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        pub_rect_.publish(msg_rect_);
      }

      if (cnn_)
      {
        ROS_ERROR_STREAM("Sleep Duration 10 seconds");
        for (int i = 0; i < 10; i++)
        {
          ros::Duration(1).sleep();
          if(!ros::ok())
            break;
        }
      }
      else
      {
        //ROS_ERROR_STREAM("Sleep Duration 2 seconds");
        for (int i = 0; i < 2; i++)
        {
          ros::Duration(1).sleep();
          if(!ros::ok())
            break;
        }
      }
      spinOnce();
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  image_publisher::ImagePublisher ip;
  ip.init();

  return 0;
}
