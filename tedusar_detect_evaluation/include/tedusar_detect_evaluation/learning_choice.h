#ifndef BAC_LEARNINGCHOICE_H
#define BAC_LEARNINGCHOICE_H

#include <tedusar_detect_evaluation/abstract_choice.h>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>

#include <vector>
#include <string>

using namespace std;
using namespace cv;

namespace learning_choice
{
  typedef struct Detection
  {
    int scoreNum;
    Size scale;
    Size patch;
    string svmModelFile;
  } Detection;

  class LearningChoice : public AbstractChoice
  {

  public:
    LearningChoice();

    virtual ~LearningChoice();

    void detect(const Mat &face, Point2f &detected, char search, const Detection &params);

    void getHogFeatures(const std::vector<cv::Mat> &patches, cv::Mat &features, cv::Size win);

    int read_num_class_data(const char *filename, int var_count, CvMat **data, CvMat **responses);

    int build_boost_classifier(const char *data_filename);

    virtual int execute();

    unsigned int getParamCnt()
    {
      return 0;
    }
  };
}

#endif //BAC_LEARNINGCHOICE_H
