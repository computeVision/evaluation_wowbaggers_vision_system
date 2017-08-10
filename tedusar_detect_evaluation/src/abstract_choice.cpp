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

#include <tedusar_detect_evaluation/abstract_choice.h>

#include <string>
#include <iostream>
#include <sstream>
#include <dirent.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

AbstractChoice::AbstractChoice()
{
}

AbstractChoice::~AbstractChoice()
{
}

int AbstractChoice::readinInt(const unsigned standard)
{
  string s = "0";
  int tmp;
  getline(cin, s);
  if (s.empty())
    return standard;
  istringstream iss(s);
  iss >> tmp;
  if (tmp == 0)
    tmp = standard;

  return tmp;
}

string AbstractChoice::readinString(const string standard)
{
  string path_ = "";
  getline(cin, path_);
  if (path_.empty())
    path_ = standard;

  return path_;
}

void AbstractChoice::swapPoints(cv::Point2i &a, cv::Point2i &b)
{
  if (a.x > b.x)
  {
    cv::Point2i tmp = a;
    a = b;
    b = tmp;
  }
}

// edited
// owner Konstantin Lassnig CollapseBuildingDetection
void AbstractChoice::readDir(std::vector<std::string> &tmp, std::string directory)
{
  DIR *dir;
  std::vector<std::string> files;

  struct dirent *entry;
  if ((dir = opendir(directory.c_str())) != NULL)
  {
    // print all the files and directories within directory
    while ((entry = readdir(dir)) != NULL)
    {
      if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
        files.push_back(std::string(entry->d_name));
    }
    closedir(dir);
    std::sort(files.begin(), files.end());
  }
  else
  {
    // could not open directory
    perror("");
    ROS_ERROR_STREAM("ERROR");
    return;
  }

  for (int i = 0; i < files.size(); i++)
  {
    ROS_INFO_STREAM("open: " << directory << " " << files.at(i));
  }

  ROS_INFO_STREAM("DONE");
  tmp = files;
}

void AbstractChoice::startScreen()
{
  cout << "Start Screen" << endl;
}

int AbstractChoice::parseImageName(std::string image_name)
{
  return stoi(image_name.substr(0, image_name.find(".")));
}
