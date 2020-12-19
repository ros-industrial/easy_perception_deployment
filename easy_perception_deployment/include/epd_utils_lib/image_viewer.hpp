// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EPD_UTILS_LIB__IMAGE_VIEWER_HPP_
#define EPD_UTILS_LIB__IMAGE_VIEWER_HPP_

#include <vector>
#include <string>
#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/opencv.hpp"

/*! \class ImageViewer
    \brief An ImageViewer class object.
    The ImageViewer class object inherits from the rclcpp::Node object to
    provide a localized way of viewing the output inference visualization
    results.
*/
class ImageViewer : public rclcpp::Node
{
public:
  /*! \brief A Constructor function*/
  ImageViewer();

private:
  /*! \brief A subscriber member variable to receive images to receive.*/
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_1_;
  /*! \brief A subscriber member variable to receive remote calls to shutdown.*/
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_2_;
  /*! \brief A Mutator function that sets the appropriate image encodings for
  displaying input images.
  */
  int encoding2mat_type(const std::string & encoding) const;
  /*! \brief A ROS2 callback function utilized by sub_1.*/
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const;
};

ImageViewer::ImageViewer()
: Node("image_viewer")
{
  cv::namedWindow("image_viewer", cv::WINDOW_AUTOSIZE);
  cv::moveWindow("image_viewer", 0, 375);
  cv::waitKey(1);

  size_t depth_ = rmw_qos_profile_default.depth;
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(history_policy_, depth_));
  qos.reliability(reliability_policy_);

  sub_1_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_viewer/image_input",
    qos, std::bind(&ImageViewer::image_callback, this, std::placeholders::_1));
}

int ImageViewer::encoding2mat_type(const std::string & encoding) const
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

void ImageViewer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
{
  cv::Mat frame(
    msg->height, msg->width, this->encoding2mat_type(msg->encoding),
    const_cast<unsigned char *>(msg->data.data()), msg->step);

  if (msg->encoding == "rgb8") {
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
  }

  cv::Mat cvframe = frame;

  cv::imshow("image_viewer", cvframe);
  cv::waitKey(1);
}

#endif  // EPD_UTILS_LIB__IMAGE_VIEWER_HPP_
