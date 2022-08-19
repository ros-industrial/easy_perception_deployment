// Copyright 2022 Advanced Remanufacturing and Technology Centre
// Copyright 2022 ROS-Industrial Consortium Asia Pacific Team
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

#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <string>
#include <memory>
#include "gtest/gtest.h"
#include "bits/stdc++.h"
#include "epd_utils_lib/epd_container.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "epd_utils_lib/easy_perception_deployment.hpp"

std::string PATH_TO_SESSION_CONFIG(PATH_TO_PACKAGE "/config/session_config.json");
std::string PATH_TO_USECASE_CONFIG(PATH_TO_PACKAGE "/config/usecase_config.json");
std::string PATH_TO_ONNX_P3_MODEL(PATH_TO_PACKAGE "/data/model/MaskRCNN-10.onnx");
std::string PATH_TO_ONNX_P2_MODEL(PATH_TO_PACKAGE "/data/model/FasterRCNN-10.onnx");
std::string PATH_TO_COCO_LABEL_LIST(PATH_TO_PACKAGE "/data/label_list/coco_classes.txt");
std::string PATH_TO_TEST_IMAGE(PATH_TO_PACKAGE "/test/nadezhda_diskant.jpg");
std::string PATH_TO_TEST_COLORED_IMAGE(PATH_TO_PACKAGE "/test/colored_img.png");
std::string PATH_TO_TEST_DEPTH_IMAGE(PATH_TO_PACKAGE "/test/depth_img.png");

TEST(EPD_TestSuite, Test_EasyPerceptionDeployment_P3Model_Tracking_Action)
{
  Json::StreamWriterBuilder builder;
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  builder["commentStyle"] = "None";
  builder["indentation"] = "    ";

  // Reset session_config.json
  system(("rm -f " + PATH_TO_SESSION_CONFIG).c_str());
  system(("touch " + PATH_TO_SESSION_CONFIG).c_str());
  // Reset usecase_config.json
  system(("rm -f " + PATH_TO_USECASE_CONFIG).c_str());
  system(("touch " + PATH_TO_USECASE_CONFIG).c_str());

  Json::Value session_config_json;
  session_config_json["path_to_model"] = PATH_TO_ONNX_P3_MODEL;
  session_config_json["path_to_label_list"] = PATH_TO_COCO_LABEL_LIST;
  session_config_json["visualizeFlag"] = "robot";
  session_config_json["useCPU"] = "CPU";

  Json::Value usecase_config_json;
  usecase_config_json["usecase_mode"] = 4;
  usecase_config_json["track_type"] = "CSRT";

  std::ofstream outputFileStream1(PATH_TO_SESSION_CONFIG);
  writer->write(session_config_json, &outputFileStream1);
  outputFileStream1.close();

  std::ofstream outputFileStream2(PATH_TO_USECASE_CONFIG);
  writer->write(usecase_config_json, &outputFileStream2);
  outputFileStream2.close();

  auto epd_node = std::make_shared<EasyPerceptionDeployment>();
  cv::Mat rgb_frame = cv::imread(PATH_TO_TEST_IMAGE, cv::IMREAD_COLOR);
  cv::Mat depth_frame = cv::imread(PATH_TO_TEST_DEPTH_IMAGE, cv::IMREAD_UNCHANGED);

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info =
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->k.at(2) = 323.3077697753906;
  camera_info->k.at(0) = 610.3740844726562;
  camera_info->k.at(5) = 235.43516540527344;
  camera_info->k.at(4) = 609.8685913085938;

  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_frame).toImageMsg();

  sensor_msgs::msg::Image::SharedPtr depth_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_frame).toImageMsg();

  epd_node->process_tracking_callback(input_msg, depth_msg, camera_info);
}

TEST(EPD_TestSuite, Test_EasyPerceptionDeployment_P3Model_Localization_Action)
{
  Json::StreamWriterBuilder builder;
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  builder["commentStyle"] = "None";
  builder["indentation"] = "    ";

  // Reset session_config.json
  system(("rm -f " + PATH_TO_SESSION_CONFIG).c_str());
  system(("touch " + PATH_TO_SESSION_CONFIG).c_str());
  // Reset usecase_config.json
  system(("rm -f " + PATH_TO_USECASE_CONFIG).c_str());
  system(("touch " + PATH_TO_USECASE_CONFIG).c_str());

  Json::Value session_config_json;
  session_config_json["path_to_model"] = PATH_TO_ONNX_P3_MODEL;
  session_config_json["path_to_label_list"] = PATH_TO_COCO_LABEL_LIST;
  session_config_json["visualizeFlag"] = "robot";
  session_config_json["useCPU"] = "CPU";

  Json::Value usecase_config_json;
  usecase_config_json["usecase_mode"] = 3;

  std::ofstream outputFileStream1(PATH_TO_SESSION_CONFIG);
  writer->write(session_config_json, &outputFileStream1);
  outputFileStream1.close();

  std::ofstream outputFileStream2(PATH_TO_USECASE_CONFIG);
  writer->write(usecase_config_json, &outputFileStream2);
  outputFileStream2.close();

  auto epd_node = std::make_shared<EasyPerceptionDeployment>();
  cv::Mat rgb_frame = cv::imread(PATH_TO_TEST_IMAGE, cv::IMREAD_COLOR);
  cv::Mat depth_frame = cv::imread(PATH_TO_TEST_DEPTH_IMAGE, cv::IMREAD_UNCHANGED);

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info =
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->k.at(2) = 323.3077697753906;
  camera_info->k.at(0) = 610.3740844726562;
  camera_info->k.at(5) = 235.43516540527344;
  camera_info->k.at(4) = 609.8685913085938;

  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_frame).toImageMsg();

  sensor_msgs::msg::Image::SharedPtr depth_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_frame).toImageMsg();

  epd_node->process_localize_callback(input_msg, depth_msg, camera_info);
}

TEST(EPD_TestSuite, Test_EasyPerceptionDeployment_P3Model_Classification_Action)
{
  Json::StreamWriterBuilder builder;
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  builder["commentStyle"] = "None";
  builder["indentation"] = "    ";

  // Reset session_config.json
  system(("rm -f " + PATH_TO_SESSION_CONFIG).c_str());
  system(("touch " + PATH_TO_SESSION_CONFIG).c_str());
  // Reset usecase_config.json
  system(("rm -f " + PATH_TO_USECASE_CONFIG).c_str());
  system(("touch " + PATH_TO_USECASE_CONFIG).c_str());

  Json::Value session_config_json;
  session_config_json["path_to_model"] = PATH_TO_ONNX_P3_MODEL;
  session_config_json["path_to_label_list"] = PATH_TO_COCO_LABEL_LIST;
  session_config_json["visualizeFlag"] = "robot";
  session_config_json["useCPU"] = "CPU";

  Json::Value usecase_config_json;
  usecase_config_json["usecase_mode"] = 0;

  std::ofstream outputFileStream1(PATH_TO_SESSION_CONFIG);
  writer->write(session_config_json, &outputFileStream1);
  outputFileStream1.close();

  std::ofstream outputFileStream2(PATH_TO_USECASE_CONFIG);
  writer->write(usecase_config_json, &outputFileStream2);
  outputFileStream2.close();

  auto epd_node = std::make_shared<EasyPerceptionDeployment>();
  cv::Mat frame = cv::imread(PATH_TO_TEST_IMAGE, cv::IMREAD_COLOR);

  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

  epd_node->process_image_callback(input_msg);
}

TEST(EPD_TestSuite, Test_EasyPerceptionDeployment_P2Model_Classification_Action)
{
  Json::StreamWriterBuilder builder;
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  builder["commentStyle"] = "None";
  builder["indentation"] = "    ";

  // Reset session_config.json
  system(("rm -f " + PATH_TO_SESSION_CONFIG).c_str());
  system(("touch " + PATH_TO_SESSION_CONFIG).c_str());
  // Reset usecase_config.json
  system(("rm -f " + PATH_TO_USECASE_CONFIG).c_str());
  system(("touch " + PATH_TO_USECASE_CONFIG).c_str());

  Json::Value session_config_json;
  session_config_json["path_to_model"] = PATH_TO_ONNX_P2_MODEL;
  session_config_json["path_to_label_list"] = PATH_TO_COCO_LABEL_LIST;
  session_config_json["visualizeFlag"] = "robot";
  session_config_json["useCPU"] = "CPU";

  Json::Value usecase_config_json;
  usecase_config_json["usecase_mode"] = 0;

  std::ofstream outputFileStream1(PATH_TO_SESSION_CONFIG);
  writer->write(session_config_json, &outputFileStream1);
  outputFileStream1.close();

  std::ofstream outputFileStream2(PATH_TO_USECASE_CONFIG);
  writer->write(usecase_config_json, &outputFileStream2);
  outputFileStream2.close();

  auto epd_node = std::make_shared<EasyPerceptionDeployment>();
  cv::Mat frame = cv::imread(PATH_TO_TEST_IMAGE, cv::IMREAD_COLOR);

  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

  epd_node->process_image_callback(input_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
