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

#include <iostream>
#include <fstream>
#include <string>
#include "gtest/gtest.h"
#include "bits/stdc++.h"
#include "epd_utils_lib/epd_container.hpp"

std::string PATH_TO_SESSION_CONFIG(PATH_TO_PACKAGE "/config/session_config.json");
std::string PATH_TO_USECASE_CONFIG(PATH_TO_PACKAGE "/config/usecase_config.json");
std::string PATH_TO_ONNX_MODEL(PATH_TO_PACKAGE "/data/model/MaskRCNN-10.onnx");
std::string PATH_TO_LABEL_LIST(PATH_TO_PACKAGE "/data/label_list/coco_classes.txt");
std::string PATH_TO_TEST_COLORED_IMAGE(PATH_TO_PACKAGE "/test/colored_img.png");
std::string PATH_TO_TEST_DEPTH_IMAGE(PATH_TO_PACKAGE "/test/depth_img.png");

TEST(EPD_TestSuite, Test_P3Model_Localize_Visualize)
{
  system(("rm -f " + PATH_TO_SESSION_CONFIG).c_str());
  system(("touch " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo " + PATH_TO_ONNX_MODEL + " >> " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo " + PATH_TO_LABEL_LIST + " >> " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo visualize >> " + PATH_TO_SESSION_CONFIG).c_str());

  system(("rm -f " + PATH_TO_USECASE_CONFIG).c_str());
  system(("touch " + PATH_TO_USECASE_CONFIG).c_str());
  system(("echo 3 >> " + PATH_TO_USECASE_CONFIG).c_str());

  EPD::EPDContainer * ortAgent_;

  ortAgent_ = new EPD::EPDContainer();

  EXPECT_EQ(ortAgent_->precision_level, unsigned(3));
  EXPECT_EQ(ortAgent_->classNames.size(), unsigned(81));
  EXPECT_EQ(ortAgent_->isVisualize(), true);

  // Download and load test image
  cv::Mat colored_img = cv::imread(PATH_TO_TEST_COLORED_IMAGE, cv::IMREAD_COLOR);
  cv::Mat depth_img = cv::imread(PATH_TO_TEST_DEPTH_IMAGE, cv::IMREAD_GRAYSCALE);

  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.k.at(2) = 323.3077697753906;
  camera_info.k.at(0) = 610.3740844726562;
  camera_info.k.at(5) = 235.43516540527344;
  camera_info.k.at(4) = 609.8685913085938;

  ortAgent_->setFrameDimension(colored_img.cols, colored_img.rows);
  ortAgent_->initORTSessionHandler();

  ASSERT_EQ(!ortAgent_->p3_ort_session, false);

  cv::Mat result = ortAgent_->p3_ort_session->infer_visualize(
    colored_img,
    depth_img,
    camera_info);

  ASSERT_NE(result.cols, 0);
  ASSERT_NE(result.rows, 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
