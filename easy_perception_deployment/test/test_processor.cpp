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

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include "gtest/gtest.h"
#include "bits/stdc++.h"
#include "epd_utils_lib/epd_container.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "epd_utils_lib/processor.hpp"

std::string PATH_TO_SESSION_CONFIG(PATH_TO_PACKAGE "/data/session_config.txt");
std::string PATH_TO_USECASE_CONFIG(PATH_TO_PACKAGE "/data/usecase_config.txt");
std::string PATH_TO_ONNX_P3_MODEL(PATH_TO_PACKAGE "/data/model/MaskRCNN-10.onnx");
std::string PATH_TO_ONNX_P2_MODEL(PATH_TO_PACKAGE "/data/model/FasterRCNN-10.onnx");
std::string PATH_TO_ONNX_P1_MODEL(PATH_TO_PACKAGE "/data/model/squeezenet1.1-7.onnx");
std::string PATH_TO_COCO_LABEL_LIST(PATH_TO_PACKAGE "/data/label_list/coco_classes.txt");
std::string PATH_TO_IMAGENET_LABEL_LIST(PATH_TO_PACKAGE "/data/label_list/imagenet_classes.txt");
std::string PATH_TO_TEST_IMAGE(PATH_TO_PACKAGE "/test/nadezhda_diskant.jpg");
std::string PATH_TO_TEST_COLORED_IMAGE(PATH_TO_PACKAGE "/test/colored_img.png");
std::string PATH_TO_TEST_DEPTH_IMAGE(PATH_TO_PACKAGE "/test/depth_img.png");

TEST(EPD_TestSuite, Test_Processor_P3Model_Classification_Action)
{
  system(("rm -f " + PATH_TO_SESSION_CONFIG).c_str());
  system(("touch " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo " + PATH_TO_ONNX_P3_MODEL + " >> " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo " + PATH_TO_COCO_LABEL_LIST + " >> " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo robot >> " + PATH_TO_SESSION_CONFIG).c_str());

  system(("rm -f " + PATH_TO_USECASE_CONFIG).c_str());
  system(("touch " + PATH_TO_USECASE_CONFIG).c_str());
  system(("echo 0 >> " + PATH_TO_USECASE_CONFIG).c_str());

  auto processor_node = std::make_shared<Processor>();
  cv::Mat frame = cv::imread(PATH_TO_TEST_IMAGE, cv::IMREAD_COLOR);

  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

  processor_node->activate_image_callback(input_msg);
}

TEST(EPD_TestSuite, Test_Processor_P2Model_Classification_Action)
{
  system(("rm -f " + PATH_TO_SESSION_CONFIG).c_str());
  system(("touch " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo " + PATH_TO_ONNX_P2_MODEL + " >> " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo " + PATH_TO_COCO_LABEL_LIST + " >> " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo robot >> " + PATH_TO_SESSION_CONFIG).c_str());

  system(("rm -f " + PATH_TO_USECASE_CONFIG).c_str());
  system(("touch " + PATH_TO_USECASE_CONFIG).c_str());
  system(("echo 0 >> " + PATH_TO_USECASE_CONFIG).c_str());

  auto processor_node = std::make_shared<Processor>();
  cv::Mat frame = cv::imread(PATH_TO_TEST_IMAGE, cv::IMREAD_COLOR);

  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

  processor_node->activate_image_callback(input_msg);
}

TEST(EPD_TestSuite, Test_Processor_P1Model_Classification_Action)
{
  system(("rm -f " + PATH_TO_SESSION_CONFIG).c_str());
  system(("touch " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo " + PATH_TO_ONNX_P1_MODEL + " >> " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo " + PATH_TO_IMAGENET_LABEL_LIST + " >> " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo robot >> " + PATH_TO_SESSION_CONFIG).c_str());

  system(("rm -f " + PATH_TO_USECASE_CONFIG).c_str());
  system(("touch " + PATH_TO_USECASE_CONFIG).c_str());
  system(("echo 0 >> " + PATH_TO_USECASE_CONFIG).c_str());

  auto processor_node = std::make_shared<Processor>();
  cv::Mat frame = cv::imread(PATH_TO_TEST_IMAGE, cv::IMREAD_COLOR);

  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

  processor_node->activate_image_callback(input_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
