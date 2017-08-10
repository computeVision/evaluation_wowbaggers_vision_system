/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Peter Lorenz
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

#include <tedusar_detect_evaluation/learning_choice.h>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/core/core.hpp" // without them cv does not work
using namespace std;
using namespace cv;

namespace learning_choice
{
  LearningChoice::LearningChoice()
  {
  }

  LearningChoice::~LearningChoice()
  {
  }

  void LearningChoice::getHogFeatures(const vector<Mat> &patches, Mat &features, Size win)
  {
    cv::HOGDescriptor hog;
    hog.winSize = win;
    hog.blockSize.height = 16;
    hog.blockSize.width = 16;
    hog.cellSize.height = 8;
    hog.cellSize.width = 8;
    hog.blockStride = Size(8, 8);

    vector<float> descriptor;
    for (const auto &patch : patches)
    {
      hog.compute(patch, descriptor);
      Mat matrix(descriptor,
                 true); //boolean value true is necessary in order to copy data // http://www.aishack.in/tutorials/opencvs-c-interface/
      transpose(matrix.clone(), matrix);
      features.push_back(matrix);
    }
  }

//================================================================================
// detect()
//--------------------------------------------------------------------------------
// TODO:
// - create SVM object using cv::CvSVM and load svm model (params.svmModelFile)
// - resize grayscaled face image to size given by params.scale

// - distinguish 3 cases according to search value:
// - if 0: search in mouth region, if 1: left eye, if 2: right eye
// - search through whole image using a sliding window with window size params.patch
// - extract hog features of each ROI using getHogFeatures()
// - use SVM object for getting distance to SVM hyperplane (cv::CvSVM::predict())
// - apply sigmoid function to returned distance
// - collect a certain number of your best scored ROIs (params.scoreNum)
// - that list of should be sorted starting with highest score_
// - apply post processing by intersecting ROIs
//   - start with first and second, keep resulting ROI
//   - intersect that ROI with next one and so on
//   - neglect resulting ROIs having less than 80% area of the previous
// - scale back that ROI to fit with original image size
// - center of last ROI is your detected object's position
// - HINT: use of cv::Rect::operator &() could be useful
//
// Mat& face: image to search in
// Point2f& detected: position of detected object to be returned (per reference)
// char search: defines searching area
// Detection& params: detection parameter
//================================================================================
  void LearningChoice::detect(const Mat &face, Point2f &detected, char search, const Detection &params)
  {
    Mat tmp_face = face.clone();
    Mat imag = face.clone();
    Mat tmp;
    CvSVM svm;
    svm.load(params.svmModelFile.c_str());

    cvtColor(imag, tmp, CV_BGR2GRAY);
    resize(tmp, imag, params.scale);

    Rect search_area;
    // - if 0: search in mouth region, if 1: left eye, if 2: right eye
    switch (search)
    {
      case 0: // Mund  // lower 40%
        search_area = Rect(0, imag.rows * 0.6, imag.cols, imag.rows * 0.4);
        break;
      case 1: // eyes
        search_area = Rect(0, 0, imag.cols * 0.5, imag.rows * 0.6);
        break;
      case 2:
        search_area = Rect(imag.cols * 0.5, 0, imag.cols * 0.5, imag.rows * 0.6);
        break;
      default:
        break;
    }

    Rect intersec;
    vector<pair<float, Rect>> vec_score;
    float dist = 0;
    // - search through whole image using a sliding window with window size params.patch
    for (int y = search_area.y;
         y < search_area.y + search_area.height - params.patch.height; y++)
    {
      for (int x = search_area.x;
           x < search_area.x + search_area.width - params.patch.width; x++)
      {
        vector<Mat> sliding_window;
        Mat mat_feature;
        sliding_window.push_back(imag(Rect(x, y, params.patch.width, params.patch.height)));

        // - extract hog features of each ROI using getHogFeatures()
        getHogFeatures(sliding_window, mat_feature, params.patch);

        // - use SVM object for getting distance to SVM hyperplane (cv::CvSVM::predict())
        dist = svm.predict(mat_feature, true);

        // - apply sigmoid function to returned distance
        dist = 1.f / (1.f + exp(-dist));

        // - collect a certain number of your best scored ROIs (params.scoreNum)
        vec_score.push_back(pair<float, Rect>(dist, Rect(x, y, params.patch.width, params.patch.height)));
      }
    }

    // - collect a certain number of your best scored ROIs (params.scoreNum)
    struct sorting : public binary_function<pair<float, Rect>, pair<float, Rect>, bool>
    {
      bool operator()(const pair<float, Rect> &__x,
                      const pair<float, Rect> &__y) const
      {
        return __x.first > __y.first;
      }
    };

    std::sort(vec_score.begin(), vec_score.end(), sorting());

    while (vec_score.size() > static_cast<unsigned>(params.scoreNum))
      vec_score.pop_back();

    // - apply post processing by intersecting ROIs
    //    - start with first and second, keep resulting ROI
    //    - intersect that ROI with next one and so on
    //    - neglect resulting ROIs having less than 80% area of the previous
    intersec = vec_score[0].second;
    Rect rec_tmp;
    for (unsigned i = 1; i < vec_score.size(); i++)
    {
      rec_tmp = intersec & vec_score[i].second;
      if (rec_tmp.area() >= intersec.area() * 0.8) // s12 80%
        intersec = rec_tmp;
    }

    // - scale back that ROI to fit with original image size
    // - center of last ROI is your detected object's position
    const float orig_height = face.rows;
    const float orig_width = face.cols;
    const float scale_height = imag.rows;
    const float scale_width = imag.cols;

    float new_scale_height = orig_height / scale_height;
    float new_scale_width = orig_width / scale_width;

    intersec.y = (float) intersec.y * new_scale_height;
    intersec.x = (float) intersec.x * new_scale_width;
    intersec.height = intersec.height * new_scale_height;
    intersec.width = intersec.width * new_scale_width;

    detected = Point2f(intersec.x + intersec.width * 0.5,
                       intersec.y + intersec.height * 0.5);
    circle(tmp_face, detected, 3, Scalar(255, 255, 255), 3, 2);
  }

  int LearningChoice::read_num_class_data(const char *filename, int var_count, CvMat **data, CvMat **responses)
  {
    const int M = 1024;
    FILE *f = fopen(filename, "rt");
    CvMemStorage *storage;
    CvSeq *seq;
    char buf[M + 2];
    float *el_ptr;
    CvSeqReader reader;
    int i, j;

    if (!f)
      return 0;

    el_ptr = new float[var_count + 1];
    storage = cvCreateMemStorage();
    seq = cvCreateSeq(0, sizeof(*seq), (var_count + 1) * sizeof(float), storage);

    for (; ;)
    {
      char *ptr;
      if (!fgets(buf, M, f) || !strchr(buf, ','))
        break;
      el_ptr[0] = buf[0];
      ptr = buf + 2;
      for (i = 1; i <= var_count; i++)
      {
        int n = 0;
        sscanf(ptr, "%f%n", el_ptr + i, &n);
        ptr += n + 1;
      }
      if (i <= var_count)
        break;
      cvSeqPush(seq, el_ptr);
    }
    fclose(f);

    *data = cvCreateMat(seq->total, var_count, CV_32F);
    *responses = cvCreateMat(seq->total, 1, CV_32F);

    cvStartReadSeq(seq, &reader);

    for (i = 0; i < seq->total; i++)
    {
      const float *sdata = (float *) reader.ptr + 1;
      float *ddata = data[0]->data.fl + var_count * i;
      float *dr = responses[0]->data.fl + i;

      for (j = 0; j < var_count; j++)
        ddata[j] = sdata[j];
      *dr = sdata[-1];
      CV_NEXT_SEQ_ELEM(seq->elem_size, reader);
    }

    cvReleaseMemStorage(&storage);
    delete[] el_ptr;
    return 1;
  }

  int LearningChoice::build_boost_classifier(const char *data_filename)
  {
    const int class_count = 26;
    CvMat *data = 0;
    CvMat *responses = 0;
    CvMat *var_type = 0;
    CvMat *temp_sample = 0;
    CvMat *weak_responses = 0;

    int ok = read_num_class_data(data_filename, 16, &data, &responses);
    int nsamples_all = 0, ntrain_samples = 0;
    int var_count;
    int i, j, k;
    double train_hr = 0, test_hr = 0;
    CvBoost boost;

    if (!ok)
    {
      printf("Could not read the database %s\n", data_filename);
      return -1;
    }

    printf("The database %s is loaded.\n", data_filename);
    nsamples_all = data->rows;
    ntrain_samples = (int) (nsamples_all * 0.5);
    var_count = data->cols;

    CvMat *new_data = cvCreateMat(ntrain_samples * class_count, var_count + 1, CV_32F);
    CvMat *new_responses = cvCreateMat(ntrain_samples * class_count, 1, CV_32S);

    // 1. unroll the database type mask
    printf("Unrolling the database...\n");
    for (i = 0; i < ntrain_samples; i++)
    {
      float *data_row = (float *) (data->data.ptr + data->step * i);
      for (j = 0; j < class_count; j++)
      {
        float *new_data_row = (float *) (new_data->data.ptr +
                                         new_data->step * (i * class_count + j));
        for (k = 0; k < var_count; k++)
          new_data_row[k] = data_row[k];
        new_data_row[var_count] = (float) j;
        new_responses->data.i[i * class_count + j] = responses->data.fl[i] == j + 'A';
      }
    }

    // 2. create type mask
    var_type = cvCreateMat(var_count + 2, 1, CV_8U);
    cvSet(var_type, cvScalarAll(CV_VAR_ORDERED));
    // the last indicator variable, as well
    // as the new (binary) response are categorical
    cvSetReal1D(var_type, var_count, CV_VAR_CATEGORICAL);
    cvSetReal1D(var_type, var_count + 1, CV_VAR_CATEGORICAL);

    // 3. train classifier
    printf("Training the classifier (may take a few minutes)...\n");
    boost.train(new_data, CV_ROW_SAMPLE, new_responses, 0, 0, var_type, 0,
                CvBoostParams(CvBoost::REAL, 100, 0.95, 5, false, 0));
    cvReleaseMat(&new_data);
    cvReleaseMat(&new_responses);
    printf("\n");

    temp_sample = cvCreateMat(1, var_count + 1, CV_32F);
    weak_responses = cvCreateMat(1, boost.get_weak_predictors()->total, CV_32F);

    // compute prediction error on train and test data
    for (i = 0; i < nsamples_all; i++)
    {
      int best_class = 0;
      double max_sum = -DBL_MAX;
      double r;
      CvMat sample;
      cvGetRow(data, &sample, i);
      for (k = 0; k < var_count; k++)
        temp_sample->data.fl[k] = sample.data.fl[k];

      for (j = 0; j < class_count; j++)
      {
        temp_sample->data.fl[var_count] = (float) j;
        boost.predict(temp_sample, 0, weak_responses);
        double sum = cvSum(weak_responses).val[0];
        if (max_sum < sum)
        {
          max_sum = sum;
          best_class = j + 'A';
        }
      }

      r = fabs(best_class - responses->data.fl[i]) < FLT_EPSILON ? 1 : 0;

      if (i < ntrain_samples)
        train_hr += r;
      else
        test_hr += r;
    }

    test_hr /= (double) (nsamples_all - ntrain_samples);
    train_hr /= (double) ntrain_samples;
    printf("Recognition rate: train = %.1f%%, test = %.1f%%\n", train_hr * 100., test_hr * 100.);

    printf("Number of trees: %d\n", boost.get_weak_predictors()->total);


    cvReleaseMat(&temp_sample);
    cvReleaseMat(&weak_responses);
    cvReleaseMat(&var_type);
    cvReleaseMat(&data);
    cvReleaseMat(&responses);

    return 0;
  }

  int LearningChoice::execute(StartScreen &init, std::vector<std::string> &params)
  {
    // http://docs.opencv.org/doc/tutorials/ml/introduction_to_svm/introduction_to_svm.html

    //  // Training data
    //  float labels[11] = { 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
    //  Mat labelsMat(11, 1, CV_32FC1, labels);
    //
    //  float trainingData[11][2] = {
    //          {501, 10}, {508, 15},
    //          {255, 10}, {501, 255}, {10, 501}, {10, 501}, {11, 501}, {9, 501}, {10, 502}, {10, 511}, {10, 495} };
    //  Mat trainingDataMat(11, 2, CV_32FC1, trainingData);
    //
    //// Set up SVM's parameters
    //  CvSVMParams paramse;
    //  paramse.svm_type    = CvSVM::C_SVC;
    //  paramse.kernel_type = CvSVM::LINEAR;
    //  paramse.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
    //
    //// Train a SVM classifier
    //  CvSVM SVM;
    //  SVM.train(trainingDataMat, labelsMat, Mat(), Mat(), paramse);
    //
    //// Train a boost classifier
    //  CvBoost boost;
    //  boost.train(trainingDataMat,
    //              CV_ROW_SAMPLE,
    //              labelsMat);
    //
    //// Test the classifiers
    //  Mat testSample1 = (Mat_<float>(1,2) << 251, 5);
    //  Mat testSample2 = (Mat_<float>(1,2) << 502, 11);
    //
    //  float svmResponse1 = SVM.predict(testSample1);
    //  float svmResponse2 = SVM.predict(testSample2);
    //
    //  float boostResponse1 = boost.predict(testSample1);
    //  float boostResponse2 = boost.predict(testSample2);
    //
    //  std::cout << "SVM:   " << svmResponse1 << " " << svmResponse2 << std::endl;
    //  std::cout << "BOOST: " << boostResponse1 << " " << boostResponse2 << std::endl;


    // build_boost_classifier( "letter-recognition.data");

    return 0;
  }
}

int main(int argc, char** argv)
{
  return 0;
}

