#include <gtest/gtest.h>
#include <tedusar_detect_evaluation/roc_choice.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/core/core.hpp" // without them cv does not work
#include <vector>

#include "ctype.h"
#include <iostream>

using namespace std;
using namespace cv;


double overlappingRect(pair <Point2i, Point2i> rect_gt, const Point2i p[2])
{
  Point2i p1, p2;
  p1.x = p[0].x;
  p1.y = p[0].y;
  p2.x = p[1].x;
  p2.y = p[1].y;

  cout << "p1.x " << p1.x << " p1.y " << p1.y << " p2.x " << p2.x << " p2.y " << p2.y << endl;
  std::pair <Point2i, Point2i> p_gt = rect_gt;
  std::pair <Point2i, Point2i> p_det(p1, p2);

  // cout << "---------------------------------------" << endl;
  // cout << "rect_det: P1: " << rect_det.first.x << " " << rect_det.first.y;
  // cout << "      P2: " << rect_det.second.x << " " << rect_det.second.y << endl;
  // cout << "rect_det: P1: ";
  // cout << rect_gt.first.x << " " << rect_gt.first.y;
  // cout << "  P2: " << rect_gt.second.x << " " << rect_gt.second.y << endl;
  // cout << "---------------------------------------" << endl;

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
    cout << "testcase" << endl;
    diff_x = p_det.second.x - p_det.first.x;
    diff_y = p_det.second.y - p_gt.first.y;
    double intersect_area = diff_x * diff_y;

    return intersect_area / det_area;
  }

  if (p_det.first.x >= p_gt.first.x and p_det.first.y < p_gt.first.y and p_det.second.x <= p_gt.second.x)
  {
    cout << "testcase" << endl;
    diff_x = p_det.second.x - p_det.first.x;
    diff_y = p_det.second.y - p_gt.first.y;
    double intersect_area = diff_x * diff_y;

    return intersect_area / det_area;
  }

  cout << "not overlapping" << endl;
  return .0;
}

TEST(Full_Overlapping_Rect, positive)
{
  Point2i a(1,1);
  Point2i b(25,25);
  pair<Point2i, Point2i> rect_gt(a, b);
//  vector<vector<unsigned>> values;
  vector<unsigned> values;

//  // roc_choice::RocChoice rc;

  values.push_back(1);
  values.push_back(1);
  values.push_back(1);
  values.push_back(1);
  values.push_back(25);
  values.push_back(25);
  Point2i val[2];
  val[0].x = 1;
  val[0].y = 1;
  val[1].x = 25;
  val[1].y = 25;
//  double tmp = overlappingRect(rect_gt, val);
  double tmp = overlappingRect(rect_gt, val);
  cout << tmp << endl;
  EXPECT_EQ(1, tmp);
}

TEST(Not_Overlapping_Rect, positive)
{
  Point2i a(1,1);
  Point2i b(25,25);
  pair<Point2i, Point2i> rect_gt(a, b);
//  vector<vector<unsigned>> values;
  vector<unsigned> values;

  values.push_back(1);
  values.push_back(1);
  values.push_back(26);
  values.push_back(26);
  values.push_back(100);
  values.push_back(100);
  Point2i val[2];
  val[0].x = 26;
  val[0].y = 26;
  val[1].x = 100;
  val[1].y = 100;

//  // roc_choice::RocChoice rc;
//  double tmp = overlappingRect(rect_gt, val);
  double tmp = overlappingRect(rect_gt, val);
  cout << tmp << endl;
  EXPECT_EQ(0, tmp);
}

// 0
TEST(Partly_LeftTOP_Overlapping, positive)
{
  Point2i a(0,0);
  Point2i b(25,25);
  pair<Point2i, Point2i> rect_gt(a, b);
//  vector<vector<unsigned>> values;
  vector<unsigned> values;

  values.push_back(1);
  values.push_back(1);
  values.push_back(15);
  values.push_back(15);
  values.push_back(100);
  values.push_back(100);
  Point2i val[2];
  val[0].x = 15;
  val[0].y = 15;
  val[1].x = 100;
  val[1].y = 100;

  // // roc_choice::RocChoice rc;
  double tmp = overlappingRect(rect_gt, val);
  cout << tmp << endl;
  EXPECT_NE(0, tmp); // 0.16
}

TEST(Partly_LeftMiddle_Overlapping, positive)
{
  Point2i a(15,15);
  Point2i b(100,100);
  pair<Point2i, Point2i> rect_gt(a, b);
//  vector<vector<unsigned>> values;
  vector<unsigned> values;

  values.push_back(1);
  values.push_back(1);
  values.push_back(0);
  values.push_back(16);
  values.push_back(25);
  values.push_back(25);
  Point2i val[2];
  val[0].x = 0;
  val[0].y = 16;
  val[1].x = 25;
  val[1].y = 25;

  // roc_choice::RocChoice rc;
  double tmp = overlappingRect(rect_gt, val);
  cout << tmp << endl;
  EXPECT_NE(0, tmp); // 0.16
}

TEST(Partly_LeftBottom_Overlapping, positive)
{
  Point2i a(25,25);
  Point2i b(100,100);
  pair<Point2i, Point2i> rect_gt(a, b);
//  vector<vector<unsigned>> values;
  vector<unsigned> values;

  values.push_back(1);
  values.push_back(1);
  values.push_back(0);
  values.push_back(90);
  values.push_back(50);
  values.push_back(120);
  Point2i val[2];
  val[0].x = 0;
  val[0].y = 90;
  val[1].x = 50;
  val[1].y = 120;

  // // roc_choice::RocChoice rc;
  double tmp = overlappingRect(rect_gt, val);
  cout << tmp << endl;
  EXPECT_NE(0, tmp); // 0.16
}

TEST(Partly_MiddleTop_Overlapping, positive)
{
  Point2i a(0,40);
  Point2i b(100,100);
  pair<Point2i, Point2i> rect_gt(a, b);
//  vector<vector<unsigned>> values;
  vector<unsigned> values;

  cout << a.x << " " << a.y << endl;

  values.push_back(1);
  values.push_back(1);
  values.push_back(25);
  values.push_back(25);
  values.push_back(50);
  values.push_back(50);
  Point2i val[2];
  val[0].x = 25;
  val[0].y = 25;
  val[1].x = 50;
  val[1].y = 50;

  // roc_choice::RocChoice rc;
  double tmp = overlappingRect(rect_gt, val);
  cout << "tmp " << tmp << endl;
  EXPECT_NE(0, tmp); // 0.16
  EXPECT_NE(1, tmp); // 0.16
}

TEST(Partly_MiddleBottom_Overlapping, positive)
{
  Point2i a(0,40);
  Point2i b(100,100);
  pair<Point2i, Point2i> rect_gt(a, b);
//  vector<vector<unsigned>> values;
  vector<unsigned> values;

  values.push_back(1);
  values.push_back(1);
  values.push_back(25);
  values.push_back(50);
  values.push_back(90);
  values.push_back(120);
  Point2i val[2];
  val[0].x = 25;
  val[0].y = 50;
  val[1].x = 90;
  val[1].y = 120;
  double tmp = overlappingRect(rect_gt, val);
  cout << tmp << endl;
  EXPECT_NE(0, tmp); // 0.16
  EXPECT_NE(1, tmp); // 0.16
}

TEST(Partly_RightMiddle_Overlapping, positive)
{
  Point2i a(25,50);
  Point2i b(120,90);
  pair<Point2i, Point2i> rect_gt(a, b);
//  vector<vector<unsigned>> values;
  vector<unsigned> values;

  values.push_back(1);
  values.push_back(1);
  values.push_back(0);
  values.push_back(40);
  values.push_back(100);
  values.push_back(100);
  Point2i val[2];
  val[0].x = 0;
  val[0].y = 40;
  val[1].x = 100;
  val[1].y = 100;

  // roc_choice::RocChoice rc;
  double tmp = overlappingRect(rect_gt, val);
  cout << tmp << endl;
  EXPECT_NE(0, tmp); // 0.16
  EXPECT_NE(1, tmp); // 0.16
}

TEST(Partly_RightTop_Overlapping, positive)
{
  Point2i a(80,30);
  Point2i b(120,80);
  pair<Point2i, Point2i> rect_gt(a, b);
//  vector<vector<unsigned>> values;
  vector<unsigned> values;

  values.push_back(1);
  values.push_back(1);
  values.push_back(0);
  values.push_back(40);
  values.push_back(100);
  values.push_back(100);
  Point2i val[2];
  val[0].x = 0;
  val[0].y = 40;
  val[1].x = 100;
  val[1].y = 100;

  // roc_choice::RocChoice rc;
  double tmp = overlappingRect(rect_gt, val);
  cout << tmp << endl;
  EXPECT_NE(0, tmp); // 0.16
  EXPECT_NE(1, tmp); // 0.16
}

TEST(Partly_RightBottom_Overlapping, positive)
{
  Point2i a(80,80);
  Point2i b(120,120);
  pair<Point2i, Point2i> rect_gt(a, b);
  // vector<vector<unsigned>> values;
  vector<unsigned> values;

  values.push_back(1);
  values.push_back(1);
  values.push_back(0);
  values.push_back(40);
  values.push_back(100);
  values.push_back(100);
  Point2i val[2];
  val[0].x = 0;
  val[0].y = 40;
  val[1].x = 100;
  val[1].y = 100;

  // roc_choice::RocChoice rc;
  double tmp = overlappingRect(rect_gt, val);
  cout << tmp << endl;
  EXPECT_NE(0, tmp); // 0.16
  EXPECT_NE(1, tmp); // 0.16
}

TEST(why0, negative)
{
  Point2i a(333,216);
  Point2i b(417,365);
  pair<Point2i, Point2i> rect_gt(a, b);
  // vector<vector<unsigned>> values;
  vector<unsigned> values;

  values.push_back(1);
  values.push_back(1);
  values.push_back(69);
  values.push_back(232);
  values.push_back(120);
  values.push_back(283);
  Point2i val[2];
  val[0].x = 69;
  val[0].y = 232;
  val[1].x = 120;
  val[1].y = 283;

  // roc_choice::RocChoice rc;
  double tmp = overlappingRect(rect_gt, val);
  cout << tmp << endl;
  EXPECT_EQ(0, tmp); // 0.16
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
