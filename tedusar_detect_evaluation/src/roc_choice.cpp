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
#include <tedusar_detect_evaluation/roc_choice.h>
#include <tedusar_detect_evaluation/my_exception.h>
#include <tedusar_detect_evaluation/RectImage.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tedusar_feature_msgs/Feature.h>
#include <tedusar_feature_msgs/SensorOutput.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <string>
#include <fstream>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include <tinyxml.h>
#include <ros/ros.h>
#include <math.h>

using namespace std;
using namespace cv;

namespace roc_choice
{
  RocChoice::RocChoice() : vec_ground_truth_(),
                           treshold_(0.3),
                           cnt_xml_(0),
                           score_("/home/jester/Dropbox/evaluation/pythonScripts/score")
  {
  }

  RocChoice::~RocChoice()
  {
  }

  void RocChoice::readDetector(std::ifstream &input, std::vector <std::vector<unsigned>> &values)
  {
    string strvalue[6];
    string line = "";
    std::stringstream ss;
    while (getline(input, line))
    {
      vector<unsigned> inner;
      std::istringstream iss(line);
      iss >> strvalue[0] >> strvalue[1] >> strvalue[2] >> strvalue[3] >> strvalue[4] >> strvalue[5];
      cout << " " << atoi(strvalue[0].c_str()) << " " << atoi(strvalue[1].c_str()) << " " <<
      atoi(strvalue[2].c_str()) << " "
      << atoi(strvalue[3].c_str()) << " " << atoi(strvalue[4].c_str()) << " " << atoi(strvalue[5].c_str()) << endl;

      for (int i = 0; i <= 5; i++)
        inner.push_back(atoi(strvalue[i].c_str()));

      values.push_back(inner);
      ss.clear();
      ss.str("");
      line.clear();
    }

    if (values.empty())
      std::cout << MyException::MESSAGES[MyException::ERROR_LOADING] << std::endl;
  }

  double RocChoice::overlappingRect(pair <Point2i, Point2i> rect_gt, const Point2i p[2])
  {
    Point2i p1, p2;
    p1.x = p[0].x;
    p1.y = p[0].y;
    p2.x = p[1].x;
    p2.y = p[1].y;

    cout << "p1.x " << p1.x << " p1.y " << p1.y << " p2.x " << p2.x << " p2.y " << p2.y << endl;
    std::pair <Point2i, Point2i> p_gt = rect_gt;
    std::pair <Point2i, Point2i> p_det(p1, p2);

    if(debug_)
    {
      // cout << "---------------------------------------" << endl;
      // cout << "rect_det: P1: " << rect_det.first.x << " " << rect_det.first.y;
      // cout << "      P2: " << rect_det.second.x << " " << rect_det.second.y << endl;
      // cout << "rect_det: P1: ";
      // cout << rect_gt.first.x << " " << rect_gt.first.y;
      // cout << "  P2: " << rect_gt.second.x << " " << rect_gt.second.y << endl;
      // cout << "---------------------------------------" << endl;
    }

    //swap to order
    if (p1.x > p2.x)
    {
      Point2i tmp = p1;
      p1 = p2;
      p2 = tmp;
    }

    if (rect_gt.first.x > rect_gt.second.x)
    {
      Point2i tmp = rect_gt.first;
      rect_gt.first = rect_gt.second;
      rect_gt.second = tmp;
    }

    //check the x coordination
    int diff_x = p_gt.second.x - p_gt.first.x;
    int diff_y = p_gt.second.y - p_gt.first.y;
    double gt_area = diff_x * diff_y;

    diff_x = p_det.second.x - p_det.first.x;
    diff_y = p_det.second.y - p_det.first.y;
    double det_area = diff_x * diff_y;
    ;

    //entirely intersecting
    if (p_det.first.x >= p_gt.first.x and p_det.first.y >= p_gt.first.y
        and p_det.second.x <= p_gt.second.x and p_det.second.y <= p_gt.second.y)
      return 1.;

    //topleftcorner of det is inside gt
    if (p_det.first.x > p_gt.first.x and p_det.first.x < p_gt.second.x
        and p_det.first.y > p_gt.first.y and p_det.first.y < p_gt.second.y)
    {
      //rightcorner be right side and above gt bottom right corner
      if (p_det.second.x > p_gt.second.x and p_det.second.y < p_gt.second.y)
      {
        diff_x = p_gt.second.x - p_det.first.x;
        diff_y = p_det.second.y - p_det.first.y;
        double intersect_area = diff_x * diff_y;

        return intersect_area / det_area ;
      }

      //bottom right corner
      if (p_det.second.x > p_gt.second.x and p_det.second.y > p_gt.second.y)
      {
        diff_x = p_gt.second.x - p_det.first.x;
        diff_y = p_gt.second.y - p_det.first.y;
        double intersect_area = diff_x * diff_y;

        return intersect_area / det_area;
      }

      //rightcorner is underneath right bottom corner
      if (p_det.second.x <= p_gt.second.x and p_det.second.y > p_gt.second.y)
      {
        diff_x = p_det.second.x - p_det.first.x;
        diff_y = p_gt.second.y - p_det.first.y;
        double intersect_area = diff_x * diff_y;

        return intersect_area / det_area;
      }
    }

    //topleft corner is on the left side, outside gt
    if (p_det.first.x < p_gt.first.x) //and p_det.second.y <= p_gt.second.y)
    {
      //topleft corner is underneath gt, outside gt and bright is inside gt < gt.2.y
      if (p_det.first.y < p_gt.first.y and p_det.second.x > p_gt.second.x
          and p_det.second.y <= p_gt.second.y)
      {
        diff_x = p_det.second.x - p_gt.first.x;
        diff_y = p_det.second.y - p_gt.first.y;
        double intersect_area = diff_x * diff_y;

        return intersect_area / det_area;
      }

      //right corner is inside gt
      if ((p_det.second.x > p_gt.first.x and p_det.second.x < p_gt.second.x)
          and p_det.second.y < p_gt.second.y)
      {
        diff_x = p_det.second.x - p_gt.first.x;
        diff_y = p_det.second.y - p_det.first.y;
        double intersect_area = diff_x * diff_y;

        return intersect_area / det_area;
      }

      //right corner is outside gt, but still parts could intersect.
      if ((p_det.second.x > p_gt.first.x and p_det.second.x < p_gt.second.x) and
          (p_det.second.y > p_gt.second.y))
      {
        diff_x = p_det.second.x - p_gt.first.x;
        diff_y = p_gt.second.y - p_det.first.y;
        double intersect_area = diff_x * diff_y;

        return intersect_area / det_area;
      }
    }

    if (p_det.first.x >= p_gt.first.x and p_det.first.y < p_gt.first.y and p_det.second.x <= p_gt.second.x)
    {
      diff_x = p_det.second.x - p_det.first.x;
      diff_y = p_det.second.y - p_gt.first.y;
      double intersect_area = diff_x * diff_y;

      return intersect_area / det_area;
    }

    if (p_det.first.x >= p_gt.first.x and p_det.first.y < p_gt.first.y and p_det.second.x <= p_gt.second.x)
    {
      diff_x = p_det.second.x - p_det.first.x;
      diff_y = p_det.second.y - p_gt.first.y;
      double intersect_area = diff_x * diff_y;

      return intersect_area / det_area;
    }

    cout << "not overlapping" << endl;
    return .0;
  }

  pair <Point2i, Point2i> RocChoice::getMinMax(std::vector <cv::Point2i> &polygon)
  {
    Point2i diff1 = polygon.at(0);
    Point2i diff2 = polygon.at(0);
    for (const auto &point : polygon)
    {
      if (point.x < diff1.x)
        diff1.x = point.x;
      if (point.y < diff1.y)
        diff1.y = point.y;

      if (point.x > diff2.x)
        diff2.x = point.x;
      if (point.y > diff2.y)
        diff2.y = point.y;
    }

    return pair<Point2i, Point2i>(diff1, diff2);
  }

  cv::Point2i RocChoice::topLeftCorner(std::vector <cv::Point2i> &point)
  {
    Point2i diff = point.at(0);
    for (const auto &po : point)
    {
      if (po.x < diff.x)
        diff.x = po.x;
      if (po.y < diff.y)
        diff.y = po.y;
    }

    return diff;
  }

  pair <Point2i, Point2i> RocChoice::parseXML(std::string xml_name)
  {
    cout << xml_name << endl;
    TiXmlDocument doc(xml_name.c_str());
    if (!doc.LoadFile())
    {
      std::cout << MyException::MESSAGES[MyException::ERROR_LOADING] + " parseXML" << std::endl;
      std::pair <Point2i, Point2i> tmp(Point2i(0, 0), Point2i(0, 0));
      return tmp;
    }

    TiXmlHandle hDoc(&doc);
    TiXmlElement *pRoot = doc.FirstChildElement("annotation");
    if (pRoot == nullptr)
    {
      std::cout << MyException::MESSAGES[MyException::TINY_XML_FAILED] << std::endl;
      cout << "No Point found!" << endl;
      std::pair <Point2i, Point2i> tmp(Point2i(0, 0), Point2i(0, 0));
      return tmp;
    }
    TiXmlElement *pObject = pRoot->FirstChildElement("object");
    if (pObject == nullptr)
    {
      std::cout << MyException::MESSAGES[MyException::TINY_XML_FAILED] << std::endl;
      cout << "No Point found!" << endl;
      std::pair <Point2i, Point2i> tmp(Point2i(0, 0), Point2i(0, 0));
      return tmp;
    }
    TiXmlElement *pPolygon = pObject->FirstChildElement("polygon");
    if (pPolygon == nullptr)
    {
      std::cout << MyException::MESSAGES[MyException::TINY_XML_FAILED] << std::endl;
      cout << "No Point found!" << endl;
      std::pair <Point2i, Point2i> tmp(Point2i(0, 0), Point2i(0, 0));
      return tmp;
    }
    TiXmlElement *pPoint = pPolygon->FirstChildElement("pt");
    if (pPoint == nullptr)
    {
      std::cout << MyException::MESSAGES[MyException::TINY_XML_FAILED] << std::endl;
      cout << "No Point found!" << endl;
      std::pair <Point2i, Point2i> tmp(Point2i(0, 0), Point2i(0, 0));
      return tmp;
    }
    std::string str_point[2];
    Point2i point;
    vector <Point2i> vec_point;
    TiXmlElement *pX;
    TiXmlElement *pY;
    while (pPoint)
    {
      pX = pPoint->FirstChildElement("x");
      pY = pPoint->FirstChildElement("y");
      if (pX == nullptr || pY == nullptr)
      {
        cout << "No Point found!" << endl;
        std::pair <Point2i, Point2i> tmp(Point2i(0, 0), Point2i(0, 0));
        return tmp;
      }

      str_point[0] = pX->GetText();
      str_point[1] = pY->GetText();
      point.x = stoi(str_point[0]);
      point.y = stoi(str_point[1]);
      vec_point.push_back(point);
      pPoint = pPoint->NextSiblingElement("pt");
    }

    return getMinMax(vec_point);
  }

  void RocChoice::writeToFile(std::ofstream& o, double percentage, bool gt_face)
  {
    if(debug_)
    {
      ROS_INFO_STREAM("writeToFile " + to_string(percentage));
      cout << "[writetofile] percentage: " << percentage << endl;
      cout << "[writetofile] bool face : " << gt_face << endl;
      cout << "[writetofile] teshold_  : " << treshold_ << endl;
    }

    cout << "----------------------------------------->" + pic_name_ << endl;

    if (percentage >= treshold_ && gt_face) //todo raise to 0.8
    {
      cout << "----True Positive----" << endl;
      o << to_string(percentage) + " TP " + pic_name_ << endl;
      lines_.push_back(to_string(percentage) + " TP");
    }
    else if (percentage > treshold_ && not gt_face)
    {
      cout << "----False Positive----" << endl;
      o << to_string(percentage) + " FP " + pic_name_  << endl;
      lines_.push_back(to_string(percentage) + " FP");
    }
    else if (percentage <= treshold_ && gt_face)
    {
      cout << "----True Negative----" << endl;
      o << to_string(percentage) + " TN " + pic_name_ << endl;
      lines_.push_back(to_string(percentage) + " TN");
    }
    else if (percentage <= treshold_ && not gt_face)
    {
      cout << "----False Negative----" << endl;
      o << to_string(percentage) + " FN " + pic_name_ << endl;
      lines_.push_back(to_string(percentage) + " FN");
    }
    else
      cout << "Nothing to write holy crap! I need tipy for my bunghole!!" << endl;
  }

  void RocChoice::run(Mat image, std::string image_name, int object[4], std::string rotation, std::vector<string> lines,
                      std::vector<pair<double, string>>& percentage)
  {
    int i_name = parseImageName(image_name);
    pair <Point2i, Point2i> rect_gt = parseXML(xml_files_ + to_string(i_name) + ".png.xml");

    if(debug_)
    {
      //    cout << "=================================" << endl;
      //    cout << object[0] << endl;
      //    cout << object[1] << endl;
      //    cout << object[2] << endl;
      //    cout << object[3] << endl;
      //    cout << "rotation: " + rotation << endl;
      //    cout << "=================================" << endl;    cout << "=================================" << endl;
    }

    Point2i p[2];
    p[0].x = object[0];
    p[0].y = object[1];
    p[1].x = object[2];
    p[1].y = object[3];
    rectangle(image, rect_gt.first,rect_gt.second, Scalar(0, 0, 255), 2, 8, 0);
    rectangle(image, p[0], p[1], Scalar(0, 255, 255), 2, 8, 0);
    imshow("roc_choice", image);

    std::ofstream of;
    of.open(score_, ios::out | ios::app);

    percentage_.push_back({overlappingRect(rect_gt, p), rotation});
    if(percentage_.size() == 4)
    {
      std::sort(percentage_.begin(), percentage_.end());
      for(auto percentages : percentage_)
        cout << "percentege: " << to_string(percentages.first) << endl;
      if(rect_gt.first.x == 0 && rect_gt.second.x == 0)
        writeToFile(of, percentage_.back().first, false); //xml is empty, no faces expected
      else
        writeToFile(of, percentage_.back().first, true);
      percentage_.clear();
    }
    //ROS_ERROR_STREAM(to_string(percentage_.size()) + " )))))))))))))");
    of.close();
  }

//  void RocChoice::imageCallbackCNN(const tedusar_feature_msgs::SensorOutputPtr& msg)
  void RocChoice::imageCallbackCNN(const tedusar_detect_evaluation::RectImagePtr& msg)
  {
    ROS_ERROR_STREAM("[rocimagecallbackcnn] " + msg->image_name);
    ROS_ERROR_STREAM("[rocimagecallbackcnn] " + to_string(msg->cnn));
    int i_name = parseImageName(msg->image_name);
    pic_name_ = msg->image_name;
    pair <Point2i, Point2i> rect_gt = parseXML(xml_files_ + to_string(i_name) + ".png.xml");
    try
    {
      Point2i p[2];
      p[0].x = msg->object1;
      p[0].y = msg->object2;
      p[1].x = msg->object3;
      p[1].y = msg->object4;

      Mat image = cv_bridge::toCvCopy(msg->sensor_msg, "bgr8")->image;
      rectangle(image, rect_gt.first,rect_gt.second, Scalar(0, 0, 255), 2, 8, 0);
      rectangle(image, p[0], p[1], Scalar(0, 255, 255), 2, 8, 0);
      imshow("roc_choice", image);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not get feature message");
    }

    std::ofstream of;
    of.open(score_, ios::out | ios::app);
    if(rect_gt.first.x == 0 && rect_gt.second.x == 0)
      writeToFile(of, msg->cnn, false); //xml is empty, no faces expected
    else
      writeToFile(of, msg->cnn, true);

    of.close();
  }

  void RocChoice::imageCallback(const tedusar_detect_evaluation::RectImagePtr& msg)
  {
    ROS_ERROR_STREAM("[roc choice]" + msg->image_name + " " + msg->rotation);
    try
    {
//      ROS_ERROR_STREAM("[roc choice] image callback!");
//      ROS_INFO_STREAM(msg->image_name);
      pic_name_ = msg->image_name;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from to 'bgr8'.");
    }

    int object[4];
    object[0] = msg->object1;
    object[1] = msg->object2;
    object[2] = msg->object3;
    object[3] = msg->object4;

    std::vector<string> line;
    run(cv_bridge::toCvCopy(msg->sensor_msg, "bgr8")->image, msg->image_name, object, msg->rotation, line, percentage_);
  }

  void RocChoice::init()
  {
    cv::namedWindow("roc_choice");
    ros::NodeHandle nh("~");

    nh.getParam("plot", plot_);
    nh.getParam("xml_files", xml_files_);
    nh.getParam("score", score_);
    nh.getParam("cnn", cnn_);
    nh.getParam("debug", debug_);
    nh.getParam("subscribe", subscribe_path_);
    nh.getParam("subscribecnn", subscribe_path_cnn_);

    cv::startWindowThread();

    std::ofstream of;
    of.open(score_, ios::out | ios::trunc);
    of << "ROC_Choice: roc curve data" << endl;
    of.close();

    ros::Subscriber sub;
    if(cnn_)
    {
      ROS_INFO_STREAM("cnn = true");
      sub = nh.subscribe(subscribe_path_cnn_, 1, &RocChoice::imageCallbackCNN, this);
    }
    else
    {
      ROS_INFO_STREAM("cnn = false");
      sub = nh.subscribe(subscribe_path_, 4, &RocChoice::imageCallback, this);
    }

    ros::spin();
    cv::destroyWindow("roc_choice");

    if(ros::ok())
    {
      cout << "Plotting ROC..." << endl;
      system(plot_.c_str());
      cout << "Plotting ROC...done" << endl;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roc_choice");
  roc_choice::RocChoice rc;
  rc.init();
  return 0;
}
