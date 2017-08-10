#ifndef BAC_ROCCHOICE_H
#define BAC_ROCCHOICE_H

#include <tedusar_detect_evaluation/abstract_choice.h>
#include <tedusar_detect_evaluation/RectImage.h>
#include <tedusar_feature_msgs/SensorOutput.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/core/core.hpp" //without them cv does not work

namespace roc_choice
{
  class DetectParams
  {
  public:
    int points_;
    cv::Size patch;
  };

  class RocChoice : public AbstractChoice
  {
    std::vector<cv::Mat> vec_ground_truth_;
    std::ifstream xml_file_;
    std::ofstream o_score_;
    cv::Point2i max_;
    cv::Point2i min_;
    double treshold_;
    unsigned cnt_xml_;
    std::vector<std::string> lines_;
    std::vector<std::pair<double, std::string>> percentage_;

    //launch file
    std::string plot_;
    std::string xml_files_;
    std::string score_;
    bool cnn_;
    bool debug_;
    std::string subscribe_path_;
    std::string subscribe_path_cnn_;

    std::string pic_name_;

  public:
    RocChoice();

    virtual ~RocChoice();

    //************************************************************************************************
    // @ param input
    // @ param values has an inner vector: imag_nr rotation p1x p1y p2x p2y
    void readDetector(std::ifstream &input, std::vector<std::vector<unsigned>> &values);

    //************************************************************************************************
    // overlappingRect
    // @ param rect_gt  The rectangle of the ground truth
    // @ param rect_det The rectangle of the current detector
    // @ return returns the percentage from 0 to 1 of how much the rect_det is overlapping the rect_gt
    double overlappingRect(std::pair <cv::Point2i, cv::Point2i> rect_gt, const cv::Point2i p[2]);

    //************************************************************************************************
    // getMinMax
    // @ param polygon around the doll face
    std::pair<cv::Point2i, cv::Point2i> getMinMax(std::vector<cv::Point2i> &polygon);

    //************************************************************************************************
    // filter from an array of points the top most left point
    cv::Point2i topLeftCorner(std::vector<cv::Point2i> &point);

    //************************************************************************************************
    // write to File
    // @ param score_ means that values for the ROC curve will be stored in a file
    // @ param percentage how much two rectangles ground truth and test case are doing intersecting
    // @ param polygon around the doll face
    // @ param gt_face says if a face is in the ground truth or not to decide whether TP, TN, FP, FN
    void writeToFile(std::ofstream& a, double percentage, bool gt_face);

    //************************************************************************************************
    // parses xml from a certain from a certain format, in order to get the ground truth polygon
    // @ param polygon
    std::pair<cv::Point2i, cv::Point2i> parseXML(std::string xml_name);

    //************************************************************************************************
    void run(cv::Mat image, std::string image_name, int object[4], std::string rotation, std::vector<std::string> line,
             std::vector<std::pair<double, std::string>>& percentage);

    //************************************************************************************************
    bool checkCnnFindings();

    //************************************************************************************************
    void imageCallbackCNN(const tedusar_detect_evaluation::RectImagePtr& msg);

    //************************************************************************************************
    void imageCallback(const tedusar_detect_evaluation::RectImagePtr& msg);

    //************************************************************************************************
    void init();

    void imagePublischerInit();
  };
}

#endif //BAC_ROCCHOICE_H
