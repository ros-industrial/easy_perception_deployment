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

#ifndef EPD_UTILS_LIB__USECASE_CONFIG_HPP_
#define EPD_UTILS_LIB__USECASE_CONFIG_HPP_

#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

/*! \brief A collection of use-case filters, namely for parsing usecase_config.txt,
Counting and Color-Matching usecaseMode.
 */
namespace EPD
{
const unsigned int CLASSIFICATION_MODE = 0;
const unsigned int COUNTING_MODE = 1;
const unsigned int COLOR_MATCHING_MODE = 2;

const char PATH_TO_USECASE_CONFIG[] = "data/usecase_config.txt";

/*! \brief A Getter function that parses the usecase_config.txt if a Counting
usecaseMode is selected and populates a list of selected object names intended
to be counted.
*/
inline std::vector<std::string> generateCountClassNames()
{
  std::vector<std::string> countClassNames;

  std::string s;
  std::fstream infile;
  infile.open(PATH_TO_USECASE_CONFIG);

  // Read and throw away the first line since it is already read.
  std::getline(infile, s);

  while (std::getline(infile, s)) {
    countClassNames.emplace_back(s);
  }
  infile.close();

  return countClassNames;
}

/*! \brief A Mutator function that takes the base inference results from a P2
inference engine and excludes any bounding boxes, classIndices and score
element that do not share the label of selected objects-to-be counted.
*/
inline void count(
  std::vector<std::array<float, 4>> & bboxes,
  std::vector<uint64_t> & classIndices,
  std::vector<float> & scores,
  std::vector<std::string> allClassNames)
{
  std::vector<std::string> countClassNames = EPD::generateCountClassNames();

  // Set max number of object to detect to 1000.
  std::vector<std::array<float, 4>> local_bboxes;
  std::vector<uint64_t> local_classIndices;
  std::vector<float> local_scores;

  /*Iterate through bbboxes, classIndices and allClassNames
  to count corresponding detected objects with the same labels.
  */
  for (size_t i = 0; i < bboxes.size(); ++i) {
    const auto & curBbox = bboxes[i];
    const uint64_t classIdx = classIndices[i];
    const float curScore = scores[i];
    const std::string curLabel = allClassNames.empty() ?
      std::to_string(classIdx) : allClassNames[classIdx];

    for (size_t j = 0; j < countClassNames.size(); j++) {
      std::string countLabel = countClassNames[j];
      if (curLabel.compare(countLabel) == 0) {
        local_bboxes.push_back(curBbox);
        local_classIndices.push_back(classIdx);
        local_scores.push_back(curScore);
      }
    }
  }

  bboxes = local_bboxes;
  classIndices = local_classIndices;
  scores = local_scores;
}

/*! \brief A Mutator function that takes the base inference results from a P2
inference engine and excludes any bounding boxes, classIndices and score
element that is not similar enough to the template color..
*/
inline void matchColor(
  const cv::Mat & img,
  std::vector<std::array<float, 4>> & bboxes,
  std::vector<uint64_t> & classIndices,
  std::vector<float> & scores,
  std::vector<std::string> allClassNames)
{
  // Get the reference image using the 2nd line of usecase_config.txt
  std::string s;
  std::fstream infile;
  infile.open(PATH_TO_USECASE_CONFIG);
  // Read and throw away the first line since it is already read.
  std::getline(infile, s);
  std::getline(infile, s);

  std::string filepath_to_refcolor = s;
  cv::Mat ref_color_image = cv::imread(filepath_to_refcolor, CV_LOAD_IMAGE_COLOR);
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
  std::vector<std::array<float, 4>> local_bboxes;
  std::vector<uint64_t> local_classIndices;
  std::vector<float> local_scores;

  for (size_t i = 0; i < bboxes.size(); ++i) {
    const auto & curBbox = bboxes[i];
    const uint64_t classIdx = classIndices[i];
    const float curScore = scores[i];
    const std::string curLabel = allClassNames.empty() ?
      std::to_string(classIdx) : allClassNames[classIdx];

    cv::Rect objectROI(cv::Point(curBbox[0], curBbox[1]), cv::Point(curBbox[2], curBbox[3]));
    croppedImage = img(objectROI);
    cv::cvtColor(croppedImage, hsv_test1, cv::COLOR_BGR2HSV);
    cv::calcHist(&hsv_test1, 1, channels, cv::Mat(),
      hist_test1, 2, histSize, ranges, true, false);
    cv::normalize(hist_test1, hist_test1, 0, 1,
      cv::NORM_MINMAX, -1, cv::Mat());

    /* Can change 3rd arg in compareHist function call to [0,1,2,3],
    [Correlation, Chi-square, Intersection, Bhattacharyya]
    TODO(cardboardcode) Require benchmark to justify use of metric 0: Correlation.*/
    base_base = compareHist(hist_base, hist_test1, 0);
    if (base_base > 0.8) {
      local_bboxes.push_back(curBbox);
      local_classIndices.push_back(classIdx);
      local_scores.push_back(curScore);
    }
  }

  bboxes = local_bboxes;
  classIndices = local_classIndices;
  scores = local_scores;
  infile.close();
}

/*! \brief A Mutator function that takes the base inference results from a P2
inference engine and excludes any bounding boxes, classIndices and score
element based on a selected use-case filter.
*/
inline void activateUseCase(
  const cv::Mat & img,
  std::vector<std::array<float, 4>> & bboxes,
  std::vector<uint64_t> & classIndices,
  std::vector<float> & scores,
  std::vector<std::string> allClassNames)
{
  unsigned int useCaseMode = 3;
  std::string s;
  std::fstream infile;
  infile.open(PATH_TO_USECASE_CONFIG);

  // Get first line of the usecase_config.txt
  std::getline(infile, s);
  if (!s.empty() && std::all_of(s.begin(), s.end(), ::isdigit)) {
    std::stringstream i(s);
    i >> useCaseMode;
  }

  // If default CLASSIFICATION_MODE is selected, do not alter anything and return.
  if (useCaseMode == EPD::CLASSIFICATION_MODE) {
    return;
  } else if (useCaseMode == EPD::COUNTING_MODE) {
    printf("Use Case: [Counting] selected.\n");
    EPD::count(bboxes, classIndices, scores, allClassNames);
  } else if (useCaseMode == EPD::COLOR_MATCHING_MODE) {
    printf("Use Case: [Color-Matching] selected.\n");
    EPD::matchColor(img, bboxes, classIndices, scores, allClassNames);
  } else {
    throw std::runtime_error("Invalid Use Case. Can only be [0, 1, 2].");
  }
}

/*! \brief A Mutator function that takes the base inference results from a P3
inference engine and excludes any bounding boxes, classIndices and score
element that do not share the label of selected objects-to-be counted.
*/
inline void count(
  std::vector<std::array<float, 4>> & bboxes,
  std::vector<uint64_t> & classIndices,
  std::vector<float> & scores,
  std::vector<cv::Mat> & masks,
  std::vector<std::string> allClassNames)
{
  std::vector<std::string> countClassNames = EPD::generateCountClassNames();

  // Set max number of object to detect to 1000.
  std::vector<std::array<float, 4>> local_bboxes;
  std::vector<uint64_t> local_classIndices;
  std::vector<float> local_scores;
  std::vector<cv::Mat> local_masks;

  /*Iterate through bbboxes, classIndices and allClassNames
  to count corresponding detected objects with the same labels.
  */
  for (size_t i = 0; i < bboxes.size(); ++i) {
    const auto & curBbox = bboxes[i];
    const uint64_t classIdx = classIndices[i];
    const float curScore = scores[i];
    const cv::Mat curMask = masks[i];
    const std::string curLabel = allClassNames.empty() ?
      std::to_string(classIdx) : allClassNames[classIdx];

    for (size_t j = 0; j < countClassNames.size(); j++) {
      std::string countLabel = countClassNames[j];
      if (curLabel.compare(countLabel) == 0) {
        local_bboxes.push_back(curBbox);
        local_classIndices.push_back(classIdx);
        local_scores.push_back(curScore);
        local_masks.push_back(curMask);
      }
    }
  }

  bboxes = local_bboxes;
  classIndices = local_classIndices;
  scores = local_scores;
  masks = local_masks;
}

/*! \brief A Mutator function that takes the base inference results from a P3
inference engine and excludes any bounding boxes, classIndices and score
element that is not similar enough to the template color..
*/
inline void matchColor(
  const cv::Mat & img,
  std::vector<std::array<float, 4>> & bboxes,
  std::vector<uint64_t> & classIndices,
  std::vector<float> & scores,
  std::vector<cv::Mat> & masks,
  std::vector<std::string> allClassNames)
{
  // Get the reference image using the 2nd line of usecase_config.txt
  std::string s;
  std::fstream infile;
  infile.open(PATH_TO_USECASE_CONFIG);
  // Read and throw away the first line since it is already read.
  std::getline(infile, s);
  std::getline(infile, s);

  std::string filepath_to_refcolor = s;
  cv::Mat ref_color_image = cv::imread(filepath_to_refcolor, CV_LOAD_IMAGE_COLOR);
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
  std::vector<std::array<float, 4>> local_bboxes;
  std::vector<uint64_t> local_classIndices;
  std::vector<float> local_scores;
  std::vector<cv::Mat> local_masks;

  for (size_t i = 0; i < bboxes.size(); ++i) {
    const auto & curBbox = bboxes[i];
    const uint64_t classIdx = classIndices[i];
    const float curScore = scores[i];
    const cv::Mat curMask = masks[i];
    const std::string curLabel = allClassNames.empty() ?
      std::to_string(classIdx) : allClassNames[classIdx];

    cv::Rect objectROI(cv::Point(curBbox[0], curBbox[1]), cv::Point(curBbox[2], curBbox[3]));
    croppedImage = img(objectROI);
    cv::cvtColor(croppedImage, hsv_test1, cv::COLOR_BGR2HSV);
    cv::calcHist(&hsv_test1, 1, channels, cv::Mat(),
      hist_test1, 2, histSize, ranges, true, false);
    cv::normalize(hist_test1, hist_test1, 0, 1,
      cv::NORM_MINMAX, -1, cv::Mat());

    /* Can change 3rd arg in compareHist function call to [0,1,2,3],
    [Correlation, Chi-square, Intersection, Bhattacharyya]
    TODO(cardboardcode) Require benchmark to justify use of metric 0: Correlation.*/
    base_base = compareHist(hist_base, hist_test1, 0);
    if (base_base > 0.8) {
      local_bboxes.push_back(curBbox);
      local_classIndices.push_back(classIdx);
      local_scores.push_back(curScore);
      local_masks.push_back(curMask);
    }
  }

  bboxes = local_bboxes;
  classIndices = local_classIndices;
  scores = local_scores;
  masks = local_masks;
  infile.close();
}

/*! \brief A Mutator function that takes the base inference results from a P3
inference engine and excludes any bounding boxes, classIndices and score
element based on a selected use-case filter.
*/
inline void activateUseCase(
  const cv::Mat & img,
  std::vector<std::array<float, 4>> & bboxes,
  std::vector<uint64_t> & classIndices,
  std::vector<float> & scores,
  std::vector<cv::Mat> & masks,
  std::vector<std::string> allClassNames)
{
  unsigned int useCaseMode = 3;
  std::string s;
  std::fstream infile;
  infile.open(PATH_TO_USECASE_CONFIG);

  // Get first line of the usecase_config.txt
  std::getline(infile, s);
  if (!s.empty() && std::all_of(s.begin(), s.end(), ::isdigit)) {
    std::stringstream i(s);
    i >> useCaseMode;
  }

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
