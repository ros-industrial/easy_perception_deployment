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

#include "bits/stdc++.h"
#include "epd_utils_lib/epd_container.hpp"
#include "gtest/gtest.h"

bool is_file_exist(const char * fileName)
{
  std::ifstream infile(fileName);
  return infile.good();
}

TEST(EPD_TestSuite, Test_loadP2ONNXModel_EPDContainer)
{
  if (!is_file_exist("./data/session_config.txt")) {
    system("touch ./data/session_config.txt");
    system("echo ./data/model/FasterRCNN-10.onnx >> ./data/session_config.txt");
    system("echo ./data/label_list/coco_classes.txt >> ./data/session_config.txt");
    system("echo robot >> ./data/session_config.txt");
  } else {
    system("rm ./data/session_config.txt");
    system("touch ./data/session_config.txt");
    system("echo ./data/model/FasterRCNN-10.onnx >> ./data/session_config.txt");
    system("echo ./data/label_list/coco_classes.txt >> ./data/session_config.txt");
    system("echo robot >> ./data/session_config.txt");
  }

  if (!is_file_exist("./data/usecase_config.txt")) {
    system("touch ./data/usecase_config.txt");
    system("echo 1 >> ./data/usecase_config.txt");
    system("echo person >> ./data/usecase_config.txt");
  } else {
    system("rm ./data/usecase_config.txt");
    system("touch ./data/usecase_config.txt");
    system("echo 1 >> ./data/usecase_config.txt");
    system("echo person >> ./data/usecase_config.txt");
  }

  EPD::EPDContainer * ortAgent_;

  ortAgent_ = new EPD::EPDContainer();

  EXPECT_EQ(ortAgent_->precision_level, unsigned(2));
  EXPECT_EQ(ortAgent_->classNames.size(), unsigned(81));

  // Download and load test image
  cv::Mat frame = cv::imread("./data/9544757988_991457c228_z.jpg", CV_LOAD_IMAGE_COLOR);

  ortAgent_->setFrameDimension(frame.cols, frame.rows);
  ortAgent_->initORTSessionHandler();

  ASSERT_EQ(!ortAgent_->p2_ort_session, false);

  EPD::EPDObjectDetection result = ortAgent_->p2_ort_session->infer_action(frame);
  ASSERT_NE(result.bboxes.size(), unsigned(0));
  ASSERT_NE(result.classIndices.size(), unsigned(0));
  ASSERT_NE(result.scores.size(), unsigned(0));

  delete ortAgent_;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
