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

#include <memory>
#include <string>
#include <fstream>
#include <functional>
#include "gtest/gtest.h"
#include "bits/stdc++.h"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "epd_utils_lib/processor.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

bool isEPDImageClassification_Received = false;

bool is_file_exist(const char * fileName)
{
  std::ifstream infile(fileName);
  return infile.good();
}

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher()
  : Node("test_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processor/image_input", 10);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&TestPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Process and publish test image as ROS message.
    cv::Mat frame = cv::imread("./data/9544757988_991457c228_z.jpg", CV_LOAD_IMAGE_COLOR);

    sensor_msgs::msg::Image::SharedPtr test_image =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

    publisher_->publish(*test_image);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

TEST(EPD_TestSuite, Test_Processor) {
  EXPECT_TRUE(isEPDImageClassification_Received);
}

int main(int argc, char ** argv)
{
  if (!is_file_exist("./data/9544757988_991457c228_z.jpg")) {
    system("apt-get install -y wget");
    system("wget "
      "https://farm8.staticflickr.com/7329/9544757988_991457c228_z.jpg "
      "--directory-prefix ./data/");
  }

  if (!is_file_exist("./data/session_config.txt")) {
    system("touch ./data/session_config.txt");
    system("echo ./data/model/FasterRCNN-10.onnx >> ./data/session_config.txt");
    system("echo ./data/label_list/coco_classes.txt >> ./data/session_config.txt");
    system("echo visualize >> ./data/session_config.txt");
  } else {
    system("rm ./data/session_config.txt");
    system("touch ./data/session_config.txt");
    system("echo ./data/model/FasterRCNN-10.onnx >> ./data/session_config.txt");
    system("echo ./data/label_list/coco_classes.txt >> ./data/session_config.txt");
    system("echo visualize >> ./data/session_config.txt");
  }

  if (!is_file_exist("./data/usecase_config.txt")) {
    system("touch ./data/usecase_config.txt");
    system("echo 0 >> ./data/usecase_config.txt");
  } else {
    system("rm ./data/usecase_config.txt");
    system("touch ./data/usecase_config.txt");
    system("echo 0 >> ./data/usecase_config.txt");
  }

  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto pub_node = std::make_shared<TestPublisher>();

  auto processor_node = std::make_shared<Processor>();

  auto sub_node = rclcpp::Node::make_shared("test_subscriber");

  // size_t depth_ = rmw_qos_profile_default.depth;
  size_t depth_ = rmw_qos_profile_default.depth;
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy_, depth_));
  qos.reliability(reliability_policy_);

  auto callback = [sub_node](const sensor_msgs::msg::Image::SharedPtr msg)
    {
      isEPDImageClassification_Received = true;
    };

  auto test_result_sub = sub_node->create_subscription<sensor_msgs::msg::Image>("/processor/output",
      qos, callback);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sub_node);
  executor.add_node(processor_node);
  executor.add_node(pub_node);

  rclcpp::WallRate loop_rate(50);
  for (int i = 0; i < 300; ++i) {
    if (!rclcpp::ok()) {
      break;  // Break for ctrl-c
    }
    executor.spin_once();
  }

  system("rm ./data/9544757988_991457c228_z.jpg");

  return RUN_ALL_TESTS();
}
