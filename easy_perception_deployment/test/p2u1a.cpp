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
#include "bits/stdc++.h"
#include "epd_utils_lib/epd_container.hpp"
#include "gtest/gtest.h"

std::string PATH_TO_SESSION_CONFIG(PATH_TO_PACKAGE "/data/session_config.txt");
std::string PATH_TO_USECASE_CONFIG(PATH_TO_PACKAGE "/data/usecase_config.txt");
std::string PATH_TO_ONNX_MODEL(PATH_TO_PACKAGE "/data/model/FasterRCNN-10.onnx");
std::string PATH_TO_LABEL_LIST(PATH_TO_PACKAGE "/data/label_list/coco_classes.txt");
std::string PATH_TO_TEST_IMAGE(PATH_TO_PACKAGE "/test/nadezhda_diskant.jpg");

TEST(EPD_TestSuite, Test_P2Model_Counting_Action)
{
  system(("rm -f " + PATH_TO_SESSION_CONFIG).c_str());
  system(("touch " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo " + PATH_TO_ONNX_MODEL + " >> " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo " + PATH_TO_LABEL_LIST + " >> " + PATH_TO_SESSION_CONFIG).c_str());
  system(("echo robot >> " + PATH_TO_SESSION_CONFIG).c_str());

  system(("rm -f " + PATH_TO_USECASE_CONFIG).c_str());
  system(("touch " + PATH_TO_USECASE_CONFIG).c_str());
  system(("echo 1 >> " + PATH_TO_USECASE_CONFIG).c_str());
  system(("echo person >> " + PATH_TO_USECASE_CONFIG).c_str());

  EPD::EPDContainer * ortAgent_;

  ortAgent_ = new EPD::EPDContainer();

  EXPECT_EQ(ortAgent_->precision_level, unsigned(2));
  EXPECT_EQ(ortAgent_->classNames.size(), unsigned(81));

  // Download and load test image
  cv::Mat frame = cv::imread(PATH_TO_TEST_IMAGE, cv::IMREAD_COLOR);

  ortAgent_->setFrameDimension(frame.cols, frame.rows);
  ortAgent_->initORTSessionHandler();

  ASSERT_EQ(!ortAgent_->p2_ort_session, false);

  EPD::EPDObjectDetection result = ortAgent_->p2_ort_session->infer_action(frame);
  ASSERT_NE(result.bboxes.size(), unsigned(0));
  ASSERT_NE(result.classIndices.size(), unsigned(0));
  ASSERT_NE(result.scores.size(), unsigned(0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
