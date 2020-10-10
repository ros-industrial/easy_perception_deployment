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
#include "gtest/gtest.h"
#include "bits/stdc++.h"
#include "epd_utils_lib/epd_container.hpp"

bool is_file_exist(const char * fileName)
{
  std::ifstream infile(fileName);
  return infile.good();
}

TEST(EPD_TestSuite, Test_emptySessionConfig_EPDContainer)
{
  if (!is_file_exist("./data/session_config.txt")) {
    system("touch ./data/session_config.txt");
    system("echo ./data/model/squeezenet1.1-7.onnx >> ./data/session_config.txt");
    system("echo ./data/label_list/imagenet_classes.txt >> ./data/session_config.txt");
  } else {
    system("rm ./data/session_config.txt");
    system("touch ./data/session_config.txt");
    system("echo ./data/model/squeezenet1.1-7.onnx >> ./data/session_config.txt");
    system("echo ./data/label_list/imagenet_classes.txt >> ./data/session_config.txt");
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

  EXPECT_EQ(ortAgent_->onnx_model_path, "./data/model/squeezenet1.1-7.onnx");
  EXPECT_EQ(ortAgent_->class_label_path, "./data/label_list/imagenet_classes.txt");

  delete ortAgent_;
}

TEST(EPD_TestSuite, Test_setFrameDimension_EPDContainer)
{
  EPD::EPDContainer * ortAgent_;
  ortAgent_ = new EPD::EPDContainer();

  ortAgent_->setFrameDimension(1920, 1080);

  EXPECT_EQ(ortAgent_->getWidth(), 1920);
  EXPECT_EQ(ortAgent_->getHeight(), 1080);

  delete ortAgent_;
}

TEST(EPD_TestSuite, Test_setInitBoolean_EPDContainer)
{
  EPD::EPDContainer * ortAgent_;
  ortAgent_ = new EPD::EPDContainer();

  EXPECT_EQ(ortAgent_->isInit(), false);
  ortAgent_->setInitBoolean(true);
  EXPECT_EQ(ortAgent_->isInit(), true);

  delete ortAgent_;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
