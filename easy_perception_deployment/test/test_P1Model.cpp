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
#include <vector>
#include "bits/stdc++.h"
#include "epd_utils_lib/epd_container.hpp"
#include "gtest/gtest.h"
// OpenCV LIB
#include "opencv2/opencv.hpp"

bool is_file_exist(const char * fileName)
{
  std::ifstream infile(fileName);
  return infile.good();
}

TEST(EPD_TestSuite, Test_loadP1ONNXModel_EPDContainer)
{
  if (!is_file_exist("./data/session_config.txt")) {
    system("touch ./data/session_config.txt");
    system("echo ./data/model/squeezenet1.1-7.onnx >> ./data/session_config.txt");
    system("echo ./data/label_list/imagenet_classes.txt >> ./data/session_config.txt");
    system("echo visualize >> ./data/session_config.txt");
  } else {
    system("rm ./data/session_config.txt");
    system("touch ./data/session_config.txt");
    system("echo ./data/model/squeezenet1.1-7.onnx >> ./data/session_config.txt");
    system("echo ./data/label_list/imagenet_classes.txt >> ./data/session_config.txt");
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

  EPD::EPDContainer * ortAgent_;

  ortAgent_ = new EPD::EPDContainer();

  EXPECT_EQ(ortAgent_->precision_level, unsigned(1));
  EXPECT_EQ(ortAgent_->classNames.size(), unsigned(1000));

  ortAgent_->setFrameDimension(1920, 1080);
  ortAgent_->initORTSessionHandler();

  ASSERT_EQ(!ortAgent_->p1_ort_session, false);

  // Download and load test image
  system("apt-get install -y wget");
  system("wget https://farm8.staticflickr.com/7329/9544757988_991457c228_z.jpg"
    " --directory-prefix ./data/");
  cv::Mat frame = cv::imread(
    "./data/9544757988_991457c228_z.jpg",
    CV_LOAD_IMAGE_COLOR);
  std::vector<std::string> topK_obj_identities = ortAgent_->p1_ort_session->infer(
    frame);

  ASSERT_EQ(topK_obj_identities[0], "Irish setter, red setter ");

  if (!is_file_exist("./data/session_config.txt")) {
    system("touch ./data/session_config.txt");
    system("echo ./data/model/squeezenet1.1-7.onnx >> ./data/session_config.txt");
    system("echo ./data/label_list/imagenet_classes.txt >> ./data/session_config.txt");
    system("echo action >> ./data/session_config.txt");
  } else {
    system("rm ./data/session_config.txt");
    system("touch ./data/session_config.txt");
    system("echo ./data/model/squeezenet1.1-7.onnx >> ./data/session_config.txt");
    system("echo ./data/label_list/imagenet_classes.txt >> ./data/session_config.txt");
    system("echo action >> ./data/session_config.txt");
  }

  topK_obj_identities = ortAgent_->p1_ort_session->infer(frame);
  ASSERT_EQ(topK_obj_identities[0], "Irish setter, red setter ");

  delete ortAgent_;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
