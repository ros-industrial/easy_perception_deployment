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

#ifndef EPD_UTILS_LIB__USECASE_CONFIG_HPP_
#define EPD_UTILS_LIB__USECASE_CONFIG_HPP_

#include <jsoncpp/json/json.h>
#include <string>
#include <vector>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "epd_utils_lib/message_utils.hpp"


/*! \brief A collection of use-case filters, namely for parsing usecase_config.json,
Counting and Color-Matching usecaseMode.
 */
namespace EPD
{
const unsigned int CLASSIFICATION_MODE = 0;
const unsigned int COUNTING_MODE = 1;
const unsigned int COLOR_MATCHING_MODE = 2;
const unsigned int LOCALISATION_MODE = 3;
const unsigned int TRACKING_MODE = 4;

const char PATH_TO_USECASE_CONFIG[] = PATH_TO_PACKAGE "/config/usecase_config.json";

/*! \brief A Getter function that parses the usecase_config.json if a Counting
usecaseMode is selected and populates a list of selected object names intended
to be counted.
*/
inline std::vector<std::string> generateCountClassNames()
{
  std::vector<std::string> countClassNames;

  Json::Reader reader;
  Json::Value obj;
  std::ifstream ifs_1(PATH_TO_USECASE_CONFIG);

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

  Json::Value class_list = obj["class_list"];
  for (int index = 0; index < static_cast<int>(class_list.size()); index++) {
    countClassNames.emplace_back(class_list[index].asString());
  }

  ifs_1.close();

  return countClassNames;
}

/*! \brief A Mutator function that takes the base inference results from a P3
inference engine and excludes any bounding boxes, classIndices and score
element that do not share the label of selected objects-to-be counted.
*/
inline void count(
  std::vector<std::array<int, 4>> & bboxes,
  std::vector<uint64_t> & classIndices,
  std::vector<float> & scores,
  std::vector<cv::Mat> & masks,
  std::vector<std::string> allClassNames)
{
  std::vector<std::string> countClassNames = EPD::generateCountClassNames();

  // Set max number of object to detect to 1000.
  std::vector<std::array<int, 4>> local_bboxes;
  std::vector<uint64_t> local_classIndices;
  std::vector<float> local_scores;
  std::vector<cv::Mat> local_masks;

  bool noMasksFound = false;
  if (masks.size() == 0) {
    noMasksFound = true;
  }

  /*Iterate through bbboxes, classIndices and allClassNames
  to count corresponding detected objects with the same labels.
  */
  for (size_t i = 0; i < bboxes.size(); ++i) {
    const auto & curBbox = bboxes[i];
    const uint64_t classIdx = classIndices[i];
    const float curScore = scores[i];
    cv::Mat curMask;
    if (!noMasksFound) {
      curMask = masks[i];
    }
    const std::string curLabel = allClassNames.empty() ?
      std::to_string(classIdx) : allClassNames[classIdx];

    for (size_t j = 0; j < countClassNames.size(); j++) {
      std::string countLabel = countClassNames[j];
      if (curLabel.compare(countLabel) == 0) {
        local_bboxes.push_back(curBbox);
        local_classIndices.push_back(classIdx);
        local_scores.push_back(curScore);
        if (!noMasksFound) {
          local_masks.push_back(curMask);
        }
      }
    }
  }

  bboxes = local_bboxes;
  classIndices = local_classIndices;
  scores = local_scores;
  if (!noMasksFound) {
    masks = local_masks;
  }
}

/*! \brief A Mutator function that takes the base inference results from a P3
inference engine and excludes any bounding boxes, classIndices and score
element that is not similar enough to the template color..
*/
inline void matchColor(
  const cv::Mat & img,
  std::vector<std::array<int, 4>> & bboxes,
  std::vector<uint64_t> & classIndices,
  std::vector<float> & scores,
  std::vector<cv::Mat> & masks,
  std::vector<std::string> allClassNames)
{
  bool noMasksFound = false;
  if (masks.size() == 0) {
    noMasksFound = true;
  }
  Json::Reader reader;
  Json::Value obj;
  std::ifstream ifs_1(PATH_TO_USECASE_CONFIG);

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

  std::string filepath_to_refcolor = obj["path_to_color_template"].asString();

  ifs_1.close();

  cv::Mat ref_color_image = cv::imread(filepath_to_refcolor, cv::IMREAD_COLOR);
  cv::Mat hsv_base, hsv_test1;
  cv::cvtColor(ref_color_image, hsv_base, cv::COLOR_BGR2HSV);
  cv::Mat hist_base, hist_test1;
  int h_bins = 50, s_bins = 60;
  int histSize[] = {h_bins, s_bins};
  int channels[] = {0, 1};

  // hue varies from 0 to 179, saturation from 0 to 255
  float h_ranges[] = {0, 180};
  float s_ranges[] = {0, 256};
  const float * ranges[] = {h_ranges, s_ranges};
  cv::calcHist(&hsv_base, 1, channels, cv::Mat(), hist_base, 2, histSize, ranges, true, false);
  cv::normalize(hist_base, hist_base, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

  double base_base;
  cv::Mat croppedImage;
  std::vector<std::array<int, 4>> local_bboxes;
  std::vector<uint64_t> local_classIndices;
  std::vector<float> local_scores;
  std::vector<cv::Mat> local_masks;

  for (size_t i = 0; i < bboxes.size(); ++i) {
    const auto & curBbox = bboxes[i];
    const uint64_t classIdx = classIndices[i];
    const float curScore = scores[i];
    cv::Mat curMask;
    if (!noMasksFound) {
      curMask = masks[i];
    }
    const std::string curLabel = allClassNames.empty() ?
      std::to_string(classIdx) : allClassNames[classIdx];

    cv::Rect objectROI(cv::Point(curBbox[0], curBbox[1]), cv::Point(curBbox[2], curBbox[3]));
    croppedImage = img(objectROI);
    cv::cvtColor(croppedImage, hsv_test1, cv::COLOR_BGR2HSV);
    cv::calcHist(
      &hsv_test1, 1, channels, cv::Mat(),
      hist_test1, 2, histSize, ranges, true, false);
    cv::normalize(
      hist_test1, hist_test1, 0, 1,
      cv::NORM_MINMAX, -1, cv::Mat());

    /* Can change 3rd arg in compareHist function call to [0,1,2,3],
    [Correlation, Chi-square, Intersection, Bhattacharyya]
    TODO(cardboardcode) Require benchmark to justify use of metric 0: Correlation.*/
    base_base = compareHist(hist_base, hist_test1, 0);
    if (base_base > 0.8) {
      local_bboxes.push_back(curBbox);
      local_classIndices.push_back(classIdx);
      local_scores.push_back(curScore);
      if (!noMasksFound) {
        local_masks.push_back(curMask);
      }
    }
  }

  bboxes = local_bboxes;
  classIndices = local_classIndices;
  scores = local_scores;
  if (!noMasksFound) {
    masks = local_masks;
  }
}

/*! \brief A Mutator function that takes the base inference results from a P3
inference engine and excludes any bounding boxes, classIndices and score
element based on a selected use-case filter.
*/
inline void activateUseCase(
  const cv::Mat & img,
  std::vector<std::array<int, 4>> & bboxes,
  std::vector<uint64_t> & classIndices,
  std::vector<float> & scores,
  std::vector<cv::Mat> & masks,
  std::vector<std::string> allClassNames)
{
  int useCaseMode = 3;

  Json::Reader reader;
  Json::Value obj;
  std::ifstream ifs_1(PATH_TO_USECASE_CONFIG);

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

  useCaseMode = obj["usecase_mode"].asInt();

  ifs_1.close();

  // If default CLASSIFICATION_MODE is selected, do not alter anything and return.
  if (useCaseMode == EPD::CLASSIFICATION_MODE) {
    return;
  } else if (useCaseMode == EPD::COUNTING_MODE) {
    printf("Use Case: [Counting] selected.\n");
    EPD::count(bboxes, classIndices, scores, masks, allClassNames);
  } else if (useCaseMode == EPD::COLOR_MATCHING_MODE) {
    printf("Use Case: [Color-Matching] selected.\n");
    EPD::matchColor(img, bboxes, classIndices, scores, masks, allClassNames);
  } else {
    throw std::runtime_error("Invalid Use Case. Can only be [0, 1, 2].");
  }
}

}  // namespace EPD

#endif  // EPD_UTILS_LIB__USECASE_CONFIG_HPP_
