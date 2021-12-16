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
  std::string filepath;
  std::fstream infile;
  infile.open(PATH_TO_SESSION_CONFIG);
  // TODO(cardboardcode) Include check if file exist or file is empty.

  while (std::getline(infile, filepath)) {
    if (filepath == "visualize") {
      onlyVisualize = true;
      break;
    }
    if (filepath == "robot") {
      onlyVisualize = false;
      break;
    }

    // Check if file exists
    if (std::ifstream(filepath)) {
      // One of the  line in the .txt file should always be for the onnx model .onnx file.
      // The other line should always be for the class label .txt file.
      if (filepath.substr(filepath.length() - 4) == "onnx") {
        onnx_model_path = filepath;
      }
      if (filepath.substr(filepath.length() - 3) == "txt") {
        class_label_path = filepath;
      }
    } else {
      std::stringstream FILE_DOES_NOT_EXIST;
      FILE_DOES_NOT_EXIST << filepath << " does not exist.";
      throw std::runtime_error(FILE_DOES_NOT_EXIST.str().c_str());
    }
  }
  infile.close();
}

void EPDContainer::setUseCaseConfigFile()
{
  std::string s;
  std::fstream infile;
  infile.open(PATH_TO_USECASE_CONFIG);

  while (std::getline(infile, s)) {
    /* Check if input string is numeric.
       If true, set the mode.
     Otherwise set the link or filter. */
    if (!s.empty() && std::all_of(s.begin(), s.end(), ::isdigit)) {
      std::stringstream i(s);
      i >> useCaseMode;
      if (useCaseMode == EPD::CLASSIFICATION_MODE) {
        break;
      }
      if (useCaseMode == EPD::LOCALISATION_MODE) {
        // Check if model precision level is not 3.
        // If true, issue critical error and close program.
        if (precision_level != 3) {
          throw std::runtime_error("Please use a Precision-Level 3 ONNX model.");
        }
        break;
      }
      if (useCaseMode > 4) {
        throw std::runtime_error("Invalid Use Case.");
      }
      continue;
    }

    if (useCaseMode == EPD::COUNTING_MODE) {
      countClassNames.emplace_back(s);
    }
    if (useCaseMode == EPD::COLOR_MATCHING_MODE) {
      template_color_path = s;
      break;
    }
    if (useCaseMode == EPD::TRACKING_MODE) {
      if (precision_level != 3) {
        throw std::runtime_error("Please use a Precision-Level 3 ONNX model.");
        break;
      }

      tracker_type = s;
      break;
    }
  }
  infile.close();

  switch (useCaseMode) {
    case EPD::CLASSIFICATION_MODE:
      printf("[-Use Case-]= EPD::CLASSIFICATION_MODE\n");
      break;
    case EPD::COUNTING_MODE:
      printf("[-Use Case-]= EPD::COUNTING_MODE\n");
      break;
    case EPD::COLOR_MATCHING_MODE:
      printf("[-Use Case-]= EPD::COLOR_MATCHING_MODE\n");
      break;
    case EPD::LOCALISATION_MODE:
      printf("[-Use Case-]= EPD::LOCALISATION_MODE\n");
      printf("[- Input RGB Image Topic -]= /camera/color/image_raw\n");
      printf(
        "[- Input Depth Image Topic -]= "
        "/camera/aligned_depth_to_color/image_raw\n");
      printf("[- Camera Info Topic -]= /camera/color/camera_info\n");
      break;
    case EPD::TRACKING_MODE:
      printf("[-Use Case-]= EPD::TRACKING_MODE\n");
      break;
  }
}

void EPDContainer::setPrecisionLevel()
{
  std::string onnx_model_filename = onnx_model_path.substr(onnx_model_path.find_last_of("/\\") + 1);
  printf("[-ONNX Model-]= %s\n", onnx_model_filename.c_str());

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
  printf("[-Precision Level-]= %d\n", precision_level);
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
  printf("[-Label List-]= %s\n", class_label_path.c_str());
}

}  // namespace EPD
