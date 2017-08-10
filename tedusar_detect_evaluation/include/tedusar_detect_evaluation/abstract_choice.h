#ifndef BAC_ABSTRACTCAPITAL_H
#define BAC_ABSTRACTCAPITAL_H

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)
#define unreachable()   __builtin_unreachable()
#define HERE() printf("Here at Line %d\n", __LINE__); sched_yield()

class AbstractChoice
{
public:
  //************************************************************************************************
  // Constructor
  // @ param letter of the choice
  AbstractChoice();

  virtual ~AbstractChoice();

  //************************************************************************************************
  // Main function
  // @ param startscreen shows an overview of the cli
  // @ param params optional additional
//  virtual int execute(StartScreen& startscreen, std::vector<std::string>& params) = 0;

  //************************************************************************************************
  // reads a number from the standard input
  // @ param standard number, e.g. count of images
  // @ return the number from stdin
  int readinInt(const unsigned standard);

  //************************************************************************************************
  // reads a string from the standard input
  std::string readinString(const std::string standard);

  //************************************************************************************************
  // swaps the points from the smallest x first
  void swapPoints(cv::Point2i& a, cv::Point2i& b);

  //************************************************************************************************
  // reads all files (pictures) from a directory
  // Owner: Konstanin Lassnig from CollapseBuildingDetection
  // @ param files names stored in a vector
  // @ param directory path to the directory to read
  void readDir(std::vector<std::string>& files, std::string directory);

  //************************************************************************************************
  void startScreen();

  //************************************************************************************************
  int parseImageName(std::string image_name);

  };

#endif //BAC_ABSTRACTCAPITAL_H
