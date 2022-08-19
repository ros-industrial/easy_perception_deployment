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

  if (ifs_1) {
    try {
      ifs_1 >> obj;
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  } else {
    std::cerr << "File not found!" << std::endl;
  }

  reader.parse(ifs_1, obj);

  onnx_model_path = obj["path_to_model"].asString();
  class_label_path = obj["path_to_label_list"].asString();

  std::string visualizeFlag = obj["visualizeFlag"].asString();

  if (visualizeFlag.compare("visualize") == 0) {
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
    for (int index = 0; index < static_cast<int>(class_list.size()); index++) {
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

cv::Mat EPDContainer::visualize(
  const EPD::EPDObjectDetection result,
  const cv::Mat input_image)
{
  // If zero objects detected, return original input image
  if (result.bboxes.size() == 0) {
    return input_image;
  }

  cv::Scalar oneColor(0.0, 0.0, 255.0, 0.0);
  cv::Mat output_image = input_image.clone();

  bool noMasksFound = false;
  cv::Mat curMask, finalMask;
  if (result.masks.size() == 0) {
    noMasksFound = true;
  }

  for (size_t i = 0; i < result.bboxes.size(); ++i) {
    const int curBbox[] = {
      result.bboxes[i][0],
      result.bboxes[i][1],
      result.bboxes[i][2],
      result.bboxes[i][3]
    };

    if (!noMasksFound) {
      curMask = result.masks[i].clone();
    }

    if (curMask.empty() && !noMasksFound) {
      continue;
    }

    if (curBbox[0] - curBbox[2] == 0) {
      continue;
    }

    if (curBbox[1] - curBbox[3] == 0) {
      continue;
    }

    const cv::Scalar & curColor = oneColor;
    const std::string curLabel = classNames[result.classIndices[i]];

    cv::rectangle(
      output_image, cv::Point(curBbox[0], curBbox[1]),
      cv::Point(curBbox[2], curBbox[3]), curColor, 2);

    int baseLine = 0;
    cv::Size labelSize =
      cv::getTextSize(curLabel, cv::FONT_HERSHEY_COMPLEX, 0.35, 1, &baseLine);
    cv::rectangle(
      output_image, cv::Point(
        curBbox[0], curBbox[1]),
      cv::Point(
        curBbox[0] + labelSize.width,
        curBbox[1] + static_cast<int>(1.3 * labelSize.height)),
      curColor, -1);

    // Visualizing masks
    const cv::Rect curBoxRect(cv::Point(curBbox[0], curBbox[1]),
      cv::Point(curBbox[2], curBbox[3]));

    if (!noMasksFound) {
      cv::resize(curMask, curMask, curBoxRect.size());
      // Assigning masks that exceed the maskThreshold.
      finalMask = (curMask > 0.5);
    }

    // Assigning coloredRoi with the bounding box.
    cv::Mat coloredRoi = (0.3 * curColor + 0.7 * output_image(curBoxRect));
    coloredRoi.convertTo(coloredRoi, CV_8UC3);

    if (!noMasksFound) {
      std::vector<cv::Mat> contours;
      cv::Mat hierarchy, tempFinalMask;
      finalMask.convertTo(tempFinalMask, CV_8U);
      cv::findContours(
        tempFinalMask, contours, hierarchy, cv::RETR_TREE,
        cv::CHAIN_APPROX_SIMPLE);
      cv::drawContours(
        coloredRoi, contours, -1, cv::Scalar(0, 0, 255), 2, cv::LINE_8,
        hierarchy, 100);
    }

    if (!noMasksFound) {
      coloredRoi.copyTo(output_image(curBoxRect), finalMask);
    }

    cv::putText(
      output_image, curLabel,
      cv::Point(curBbox[0], curBbox[1] + labelSize.height),
      cv::FONT_HERSHEY_COMPLEX, 0.35, cv::Scalar(255, 255, 255));
  }

  return output_image;
}

cv::Mat EPDContainer::visualize(
  const EPD::EPDObjectTracking result,
  const cv::Mat input_image)
{
  // If zero objects detected, return original input image
  if (result.objects.size() == 0) {
    return input_image;
  }

  cv::Scalar oneColor(0.0, 0.0, 255.0, 0.0);

  cv::Mat output_image = input_image.clone();
  for (size_t i = 0; i < result.objects.size(); ++i) {
    const unsigned int curBbox[] = {
      result.objects[i].roi.x_offset,
      result.objects[i].roi.y_offset,
      result.objects[i].roi.width + result.objects[i].roi.x_offset,
      result.objects[i].roi.height + result.objects[i].roi.y_offset};
    cv::Mat curMask = result.objects[i].mask.clone();

    if (curMask.empty()) {
      continue;
    }

    if (curBbox[0] - curBbox[2] == 0) {
      continue;
    }

    if (curBbox[1] - curBbox[3] == 0) {
      continue;
    }

    const cv::Scalar & curColor = oneColor;
    std::string curLabel = result.objects[i].name;

    cv::rectangle(
      output_image,
      cv::Point(curBbox[0], curBbox[1]),
      cv::Point(curBbox[2], curBbox[3]),
      curColor,
      2);

    int baseLine = 0;
    cv::Size labelSize =
      cv::getTextSize(curLabel, cv::FONT_HERSHEY_COMPLEX, 0.35, 1, &baseLine);
    cv::rectangle(
      output_image, cv::Point(curBbox[0], curBbox[1]),
      cv::Point(
        curBbox[0] + labelSize.width,
        curBbox[1] + static_cast<int>(1.3 * labelSize.height)),
      curColor, -1);

    if (result.object_ids.size() != 0) {
      curLabel = curLabel + "_" + result.object_ids[i];
    }

    // Visualizing masks
    const cv::Rect curBoxRect(cv::Point(curBbox[0], curBbox[1]),
      cv::Point(curBbox[2], curBbox[3]));
    cv::resize(curMask, curMask, curBoxRect.size());
    // Assigning masks that exceed the maskThreshold.
    cv::Mat finalMask = (curMask > 0.5);

    // Assigning coloredRoi with the bounding box.
    cv::Mat coloredRoi = (0.3 * curColor + 0.7 * output_image(curBoxRect));
    coloredRoi.convertTo(coloredRoi, CV_8UC3);

    std::vector<cv::Mat> contours;
    cv::Mat hierarchy, tempFinalMask;
    finalMask.convertTo(tempFinalMask, CV_8U);
    cv::findContours(
      tempFinalMask, contours, hierarchy, cv::RETR_TREE,
      cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(
      coloredRoi, contours, -1, cv::Scalar(0, 0, 255), 2, cv::LINE_8,
      hierarchy, 100);

    coloredRoi.copyTo(output_image(curBoxRect), finalMask);

    cv::putText(
      output_image, curLabel,
      cv::Point(curBbox[0], curBbox[1] + labelSize.height),
      cv::FONT_HERSHEY_COMPLEX, 0.35, cv::Scalar(255, 255, 255));
  }

  return output_image;
}
}  // namespace EPD
