#ifndef IMAGE_TRANSPORT_TUTORIAL_MY_PUBLISHER_H
#define IMAGE_TRANSPORT_TUTORIAL_MY_PUBLISHER_H

#include <tedusar_detect_evaluation/abstract_choice.h>
#include <tedusar_feature_msgs/Feature.h>
#include <tedusar_feature_msgs/SensorOutput.h>
#include <sensor_msgs/image_encodings.h>
#include <tedusar_detect_evaluation/RectImage.h>

#include <ros/ros.h>

namespace image_publisher
{
  class ImagePublisher : AbstractChoice {
  private:
    std::string dir_;
    bool debug_;
    bool cnn_;
    std::string subscribe_path_;
    std::string pub_rect_path_;
    std::string pub_roc_path_;
    std::string pub_cnn_path_;

    ros::NodeHandle nh_;
    ros::Publisher pub_rect_;
    ros::Publisher pub_roc_;
    ros::Publisher pub_cnn_;
    ros::Subscriber sub_;

    tedusar_detect_evaluation::RectImage msg_rect_;

  public:
    //************************************************************************************************
    ImagePublisher();

    //************************************************************************************************
    virtual ~ImagePublisher();

    //************************************************************************************************
    void init();

    //************************************************************************************************
    void imageCallCnn(const tedusar_feature_msgs::SensorOutputPtr &msg);

    void imageCall(const tedusar_detect_evaluation::RectImagePtr& msg);
  };
}
#endif //IMAGE_TRANSPORT_TUTORIAL_MY_PUBLISHER_H
