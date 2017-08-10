#ifndef BAC_WOWBAGGERCHOICE_H
#define BAC_WOWBAGGERCHOICE_H

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/core/core.hpp" //without them cv does not work
#include <tedusar_detect_evaluation/abstract_choice.h>
#include <tedusar_detect_evaluation/RectImage.h>

#include <string>
#include <iostream>
#include <fstream>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>


namespace overlapping_rect
{
  class WowbaggerChoice : AbstractChoice {
    std::string face_cascade_name_;
    std::string eyes_cascade_name_;
    std::string subscribe_path_;
    std::string time_measurement_;
    int width_;
    int height_;
    int input_images_;
    ros::Publisher pub;
    ros::NodeHandle nh;
    tedusar_detect_evaluation::RectImage msg_;

  public:
    //************************************************************************************************
    // Constructor
    WowbaggerChoice();

    //************************************************************************************************
    // Destructor
    virtual ~WowbaggerChoice();

    //************************************************************************************************
    // Constructor
    // @ param src is a image frame which will be rotated
    // @ param angle in degree, how much the frame should be rotated clockwise
    cv::Mat rotate(cv::Mat &src, double angle);

    //************************************************************************************************
    // detect and display
    // @ param frame that is investigated
    // @ param counter which image we have
    // @ param letter a: not rotated, b: 90degree, c: 180degree, d: 270degree
    void detectAndDisplay(cv::Mat &frame, cv::Mat& origin, int counter, char letter);

    //************************************************************************************************
    // rotate Point around a center (0,0)
    // @ param point
    // @ param rad - radians, positive means counter-clockwise
    // @ return p'
    cv::Point2i rotatePoint(cv::Point2i& point, double rad);

    //************************************************************************************************
    cv::Point2i rotatePointBack(cv::Point2i& point, double rad);

    //************************************************************************************************
    cv::Point2i rotatePointCenter(const cv::Point2i& cen_pt, const cv::Point2i& p, double rad);

    //************************************************************************************************
    // run
    // @ param input a frame subscribed by init
    // @ param image_name
    void run(cv::Mat input, std::string image_name, const tedusar_detect_evaluation::RectImagePtr& msg, std::ofstream& of);
//    void run(cv::Mat input, std::string image_name, const tedusar_detect_evaluation::RectImagePtr& msg);


    void imageCallback(const tedusar_detect_evaluation::RectImagePtr& msg);


    //************************************************************************************************
    // init - the ros system to subscribe pictures by a publisher
    void init();

    //************************************************************************************************
    // getters and setters
    unsigned getWidth() const
    {
      return width_;
    }

    unsigned getHeight() const
    {
      return height_;
    }

    unsigned getImageCount() const
    {
      return input_images_;
    }
  };
}
#endif //BAC_WOWBAGGERCHOICE_H
