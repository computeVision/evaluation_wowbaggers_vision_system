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

#include "tedusar_detect_evaluation/overlapping_rect.h"
#include "tedusar_detect_evaluation/my_exception.h"
#include <tedusar_detect_evaluation/RectImage.h>

#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>


namespace overlapping_rect
{
  cv::CascadeClassifier face_cascade_;
  cv::CascadeClassifier eyes_cascade_;

  using namespace cv;
  using namespace std;

  WowbaggerChoice::WowbaggerChoice() : subscribe_path_("/tooverlapping"),
                                       face_cascade_name_("/home/jester/Dropbox/evaluation/lib/haarcascade_frontalface_alt.xml"),
                                       eyes_cascade_name_("/home/jester/Dropbox/evaluation/lib/haarcascade_eye_tree_eyeglasses.xml"),
                                       time_measurement_("/home/jester/Dropbox/evaluation/pythonScripts/time_measurment_haarcascade"),
                                       width_(640),
                                       height_(480),
                                       input_images_(0),
                                       nh("~"),
                                       msg_()
  {
    ROS_WARN_STREAM("overlapping rect ctor");
    pub = nh.advertise<tedusar_detect_evaluation::RectImage>("/toimagepublisher", 4);
  }

  WowbaggerChoice::~WowbaggerChoice()
  {
  }

  Mat WowbaggerChoice::rotate(Mat& src, double angle = 90)
  {
    Mat dst;
    Point2f pt(src.cols * .5, src.rows *.5);
    cv::transpose(src, dst);
    cv::flip(dst, dst, 1);

    return dst;
  }

  void WowbaggerChoice::detectAndDisplay(cv::Mat& frame, cv::Mat& origin, int counter, char rotation)
  {
    //http://docs.opencv.org/2.4.10/doc/tutorials/imgproc/shapedescriptors/bounding_rects_circles/bounding_rects_circles.html
    std::vector<Rect> faces;
    Mat frame_gray;
    cvtColor(frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    face_cascade_.detectMultiScale( frame_gray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));
    ROS_WARN_STREAM("faces size " + to_string(faces.size()));

    Rect max_rect;
    int tmp_i = 0;

    if(faces.empty() || !faces.size())
    {
      msg_.rotation = rotation;
      msg_.object1 = 0;
      msg_.object2 = 0;
      msg_.object3 = 0;
      msg_.object4 = 0;
    }
    else
    {
      ROS_ERROR_STREAM("face size overlappint rect");
      //take the biggest one
      for(size_t i = 0; i < faces.size(); i++)
      {
        if (faces[i].area() > max_rect.area())
        {
          max_rect = faces[i];
          tmp_i = i;
        }
      }

      rectangle(frame, faces[tmp_i], Scalar(0, 255, 255), 2, 8, 0);
      imshow("overlapping_rect", frame);

//      cout << "-----------------------------------" << endl;
//      cout << "x " << faces[0].x << "y " << faces[0].y << endl;
//      cout << "-----------------------------------" << endl;

      // check if out of boundary
      if (faces[tmp_i].x + 100 > width_)
        faces[tmp_i].x -= faces[tmp_i].x + faces[tmp_i].width - width_;
      if (faces[tmp_i].y + 100 > height_)
        faces[0].y -= faces[0].y + faces[tmp_i].height - height_;
//      cout << "f x1:     " << faces[tmp_i].x << endl;
//      cout << "f y1:     " << faces[tmp_i].y << endl;
//      cout << "f width:  " << faces[tmp_i].width << endl;
//      cout << "f height: " << faces[tmp_i].height << endl;

      cv::Point2i bottom_right_corner;
      bottom_right_corner.x = faces[tmp_i].x + faces[tmp_i].width;
      bottom_right_corner.y = faces[tmp_i].y + faces[tmp_i].height;

      Point2i p[2];
      p[0].x = faces[tmp_i].x;
      p[0].y = faces[tmp_i].y;
      p[1].x = p[0].x + faces[tmp_i].width; //x + width
      p[1].y = p[0].y + faces[tmp_i].height; //y + height

//    ROS_ERROR_STREAM("======================");
//    ROS_ERROR_STREAM(rotation);
      switch (rotation)
      {
        case 'b': //90degree
        {
          Point2i tmp;
          for (int i = 0; i < sizeof(p) / sizeof(p[0]); i++)
          {
            tmp.x = p[i].y;
            tmp.y = frame.size().width - p[i].x;
            p[i] = tmp;
          }
          break;
        }
        case 'c': //180degree
        {
          Point2i tmp;
          for (int i = 0; i < sizeof(p) / sizeof(p[0]); i++)
          {
            tmp.y = frame.size().height - p[i].y;
            tmp.x = frame.size().width - p[i].x;
            p[i] = tmp;
          }
          break;
        }
        case 'd': //270degree
        {
          Point2i tmp;
          for (int i = 0; i < sizeof(p) / sizeof(p[0]); i++)
          {
            tmp.y = p[i].x;
            tmp.x = frame.size().height - p[i].y;
            p[i] = tmp;
          }
          break;
        }
        default:
          break;
      }

      //publish image
      msg_.rotation = rotation;
      msg_.object1 = p[0].x;
      msg_.object2 = p[0].y;
      msg_.object3 = p[1].x;
      msg_.object4 = p[1].y;
    }

    rotation = 0; //reset
    pub.publish<tedusar_detect_evaluation::RectImage>(msg_);
    //ros::Duration(2).sleep();
    ros::spinOnce();
  }

  cv::Point2i WowbaggerChoice::rotatePoint(cv::Point2i& point, double rad)
  {
    const double x = cos(rad) * point.x - sin(rad) * point.y;
    const double y = sin(rad) * point.x + cos(rad) * point.y;

    const cv::Point2f rot_p(x, y);
    return rot_p;
  }

  cv::Point2i WowbaggerChoice::rotatePointBack(cv::Point2i& point, double rad)
  {
    float dist = (point.x * point.x) + (point.y * point.y);
    point.x = sqrt(dist)*sin(rad*M_PI/180);
    point.y = sqrt(dist)*cos(rad*M_PI/180);

    return point;
  }

  cv::Point2i WowbaggerChoice::rotatePointCenter(const cv::Point2i& cen_pt, const cv::Point2i& p, double rad)
  {
    cv::Point2i trans_pt = p - cen_pt;
    const cv::Point2i rot_pt   = rotatePoint(trans_pt, rad);
    const cv::Point2i fin_pt   = rot_pt + cen_pt;

    return fin_pt;
  }

  void WowbaggerChoice::run(Mat input, std::string image_name, const tedusar_detect_evaluation::RectImagePtr& msg, std::ofstream& of)
  {
    msg_ = *msg;
    int i = parseImageName(image_name);

    double begin = ros::Time::now().toSec();
    Mat dst;
    detectAndDisplay(input, input, i, 'a');
    dst = rotate(input);
    detectAndDisplay(dst, input, i, 'b');
    dst = rotate(dst);
    detectAndDisplay(dst, input, i, 'c');
    dst = rotate(dst);
    detectAndDisplay(dst, input, i, 'd');
    double end = ros::Time::now().toSec();

    cout << "result time " << end - begin << endl;
    of << end - begin << endl;
  }

  void WowbaggerChoice::imageCallback(const tedusar_detect_evaluation::RectImagePtr& msg)
  {
    try
    {
      ROS_INFO_STREAM("Overlapping rect callback");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from to 'bgr8'.");
    }
    //ROS_INFO_STREAM("overlapping rect");
    std::ofstream of;
    of.open(time_measurement_, ios::out | ios::app);
    run(cv_bridge::toCvCopy(msg->sensor_msg, "bgr8")->image, msg->image_name, msg, of);
    of.close();
  }


  void WowbaggerChoice::init()
  {
    cv::namedWindow("overlapping_rect");
    cv::startWindowThread();

    nh.getParam("face_cascade", face_cascade_name_);
    nh.getParam("eye_cascade", eyes_cascade_name_);
    nh.getParam("subscribe_path", subscribe_path_);

    std::ofstream of;
    of.open(time_measurement_, ios::out | ios::trunc);
    of << "TIME_HAARCASCADE:" << endl;
    of.close();
    if(!face_cascade_.load(face_cascade_name_))
    {
      std::cout << MyException::MESSAGES[MyException::ERROR_LOADING] + face_cascade_name_<< std::endl;
      return ;
    }

    if(!eyes_cascade_.load(eyes_cascade_name_) )
    {
      std::cout << MyException::MESSAGES[MyException::ERROR_LOADING] + eyes_cascade_name_ << std::endl;
      return ;
    }

    ros::Subscriber sub = nh.subscribe(subscribe_path_, 1, &WowbaggerChoice::imageCallback, this);

    ros::spin();
    cv::destroyWindow("overlapping_rect");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");

  overlapping_rect::WowbaggerChoice wc;
  wc.init();

  return 0;
}
