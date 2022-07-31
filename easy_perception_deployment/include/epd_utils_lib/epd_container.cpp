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

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include "epd_container.hpp"
#include "epd_utils_lib/usecase_config.hpp"
#include <jsoncpp/json/json.h>
#include "rclcpp/rclcpp.hpp"


namespace EPD
{

EPDContainer::EPDContainer(void)
{
  hasInitialized = false;
  onlyVisualize = true;

  this->setModelConfigFile();
  this->setPrecisionLevel();
  this->setLabelList();
  this->setUseCaseConfigFile();
}

EPDContainer::~EPDContainer() {}

bool EPDContainer::isInit(void)
{
  return hasInitialized;
}

bool EPDContainer::isVisualize(void)
{
  return onlyVisualize;
}

bool EPDContainer::isService(void)
{
  return onlyService;
}

int EPDContainer::getHeight() {return frame_height;}

int EPDContainer::getWidth() {return frame_width;}

void EPDContainer::setInitBoolean(bool input)
{
  hasInitialized = input;
}

void EPDContainer::setFrameDimension(int input_width, int input_height)
{
  frame_width = input_width;
  frame_height = input_height;
}

void EPDContainer::initORTSessionHandler()
{
  float ratio = 800.0 / std::min(frame_width, frame_height);
  int newW = ratio * frame_width;
  int newH = ratio * frame_height;
  // Ensure that padded dimensions are divisible by 32.
  int paddedW = static_cast<int>(((newW + 31) / 32) * 32);
  int paddedH = static_cast<int>(((newH + 31) / 32) * 32);

  switch (precision_level) {
    case 1:
      p1_ort_session = new Ort::P1OrtBase(
        ratio, 224, 224, paddedW, paddedH,
        classNames.size(),
        onnx_model_path,
        0,
        std::vector<std::vector<int64_t>>{{1, IMG_CHANNEL, 224, 224}}
      );
      p1_ort_session->initClassNames(classNames);
      break;
    case 2:
      p2_ort_session = new Ort::P2OrtBase(
        ratio, newW, newH, paddedW, paddedH,
        classNames.size(),
        onnx_model_path,
        0,
        std::vector<std::vector<int64_t>>{{IMG_CHANNEL, paddedH, paddedW}}
      );
      p2_ort_session->initClassNames(classNames);
      break;
    case 3:
      p3_ort_session = new Ort::P3OrtBase(
        ratio, newW, newH, paddedW, paddedH,
        classNames.size(),
        onnx_model_path,
        0,
        std::vector<std::vector<int64_t>>{{IMG_CHANNEL, paddedH, paddedW}}
      );
      p3_ort_session->initClassNames(classNames);
      break;
  }
}

void EPDContainer::setModelConfigFile()
{
  Json::Reader reader;
  Json::Value obj;
  std::ifstream ifs_1(PATH_TO_SESSION_CONFIG);
  reader.parse(ifs_1, obj);

  onnx_model_path = obj["path_to_model"].asString();
  class_label_path = obj["path_to_label_list"].asString();

  std::string visualizeFlag = obj["visualizeFlag"].asString();

  if (visualizeFlag.compare("visualize") == 0){
      onlyVisualize = true;
  } else {
      onlyVisualize = false;
  }

  ifs_1.close();

}

void EPDContainer::setUseCaseConfigFile()
{
  Json::Reader reader;
  Json::Value obj;
  std::ifstream ifs_1(PATH_TO_USECASE_CONFIG);
  reader.parse(ifs_1, obj);

  useCaseMode = obj["usecase_mode"].asInt();

  // Classification Mode. Do nothing.
  // Counting Mode
  if (useCaseMode == EPD::COUNTING_MODE) {
    Json::Value class_list = obj["class_list"];
    for ( size_t index = 0; index < class_list.size(); index++ )
    {
        countClassNames.emplace_back(class_list[index].asString());
    }
  }

  if (useCaseMode == EPD::COLOR_MATCHING_MODE) {
    template_color_path = obj["path_to_color_template"].asString();
  }

  // Localization Mode
  if (useCaseMode == EPD::LOCALISATION_MODE) {
    // Check if model precision level is not 3.
    // If true, issue critical error and close program.
    if (precision_level != 3) {
      throw std::runtime_error("Please use a Precision-Level 3 ONNX model.");
    }
  }

  // Tracking Mode
  if (useCaseMode == EPD::TRACKING_MODE) {
    // Check if model precision level is not 3.
    // If true, issue critical error and close program.
    if (precision_level != 3) {
      throw std::runtime_error("Please use a Precision-Level 3 ONNX model.");
    }
    tracker_type = obj["track_type"].asString();
  }

  // Invalid Use Case Mode
  if (useCaseMode > 4) {
    throw std::runtime_error("Invalid Use Case.\n");
  }

  ifs_1.close();
}

void EPDContainer::setPrecisionLevel()
{
  std::vector<std::vector<int64_t>> empty_inputShapes;

  Ort::OrtBase ort_session(onnx_model_path, 0, empty_inputShapes);

  switch (ort_session.getNumOutputs()) {
    case 1:
      precision_level = 1;
      break;
    case 3:
      precision_level = 2;
      break;
    case 4:
      precision_level = 3;
      break;
    default:
      throw std::runtime_error("Invalid Precision Level. Report as GitHub issue.");
  }
}

void EPDContainer::setLabelList()
{
  std::string label;
  std::fstream infile;
  infile.open(class_label_path);

  while (std::getline(infile, label)) {
    classNames.emplace_back(label);
  }

  infile.close();
}

}  // namespace EPD
