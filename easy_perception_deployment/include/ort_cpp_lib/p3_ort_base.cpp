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
#include <cstring>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

#include "p3_ort_base.hpp"
#include "epd_utils_lib/usecase_config.hpp"
#include "tf2/LinearMath/Quaternion.h"


namespace Ort
{
// Constructor
P3OrtBase::P3OrtBase(
  float ratio,
  int newW,
  int newH,
  int paddedW,
  int paddedH,
  const uint16_t numClasses,
  const std::string & modelPath,
  const boost::optional<size_t> & gpuIdx,
  const boost::optional<std::vector<std::vector<int64_t>>> & inputShapes)
: OrtBase(modelPath, gpuIdx, inputShapes),
  m_numClasses(numClasses),
  m_ratio(ratio),
  m_newW(newW),
  m_newH(newH),
  m_paddedW(paddedW),
  m_paddedH(paddedH)
{}

// Destructor
P3OrtBase::~P3OrtBase()
{}

// Mutator 4
cv::Mat P3OrtBase::infer_visualize(const cv::Mat & inputImg)
{
  std::vector<float> dst(3 * m_paddedH * m_paddedW);

  return this->infer_visualize(
    inputImg, m_newW, m_newH,
    m_paddedW, m_paddedH, m_ratio, dst.data(), 0.5,
    cv::Scalar(102.9801, 115.9465, 122.7717));
}

// Mutator 4
cv::Mat P3OrtBase::infer_visualize(
  const cv::Mat & inputImg,
  const cv::Mat & depthImg,
  sensor_msgs::msg::CameraInfo camera_info)
{
  std::vector<float> dst(3 * m_paddedH * m_paddedW);

  return this->infer_visualize(
    inputImg, depthImg, camera_info, m_newW, m_newH,
    m_paddedW, m_paddedH, m_ratio, dst.data(), 0.5,
    cv::Scalar(102.9801, 115.9465, 122.7717));
}

// Mutator 4
EPD::EPDObjectDetection P3OrtBase::infer_action(const cv::Mat & inputImg)
{
  std::vector<float> dst(3 * m_paddedH * m_paddedW);

  return this->infer_action(
    inputImg, m_newW, m_newH,
    m_paddedW, m_paddedH, m_ratio, dst.data(), 0.5,
    cv::Scalar(102.9801, 115.9465, 122.7717));
}

// Mutator 4
EPD::EPDObjectLocalization P3OrtBase::infer_action(
  const cv::Mat & inputImg,
  const cv::Mat & depthImg,
  sensor_msgs::msg::CameraInfo camera_info)
{
  std::vector<float> dst(3 * m_paddedH * m_paddedW);

  return this->infer_action(
    inputImg, depthImg, camera_info, m_newW, m_newH,
    m_paddedW, m_paddedH, m_ratio, dst.data(), 0.5,
    cv::Scalar(102.9801, 115.9465, 122.7717));
}

void P3OrtBase::initClassNames(const std::vector<std::string> & classNames)
{
  if (classNames.size() != m_numClasses) {
    throw std::runtime_error("Mismatch number of classes\n");
  }
  m_classNames = classNames;
}

void P3OrtBase::preprocess(
  float * dst,
  const cv::Mat & imgSrc,
  const int64_t targetImgWidth,
  const int64_t targetImgHeight,
  const int numChannels) const
{
  for (int i = 0; i < targetImgHeight; ++i) {
    for (int j = 0; j < targetImgWidth; ++j) {
      for (int c = 0; c < numChannels; ++c) {
        dst[c * targetImgHeight * targetImgWidth +
          i * targetImgWidth + j] =
          imgSrc.ptr<float>(i, j)[c];
      }
    }
  }
}

// Mutator 4
cv::Mat P3OrtBase::infer_visualize(
  const cv::Mat & inputImg,
  int newW,
  int newH,
  int paddedW,
  int paddedH,
  float ratio,
  float * dst,
  float confThresh,
  const cv::Scalar & meanVal)
{
  cv::Mat tmpImg;
  cv::resize(inputImg, tmpImg, cv::Size(newW, newH));

  tmpImg.convertTo(tmpImg, CV_32FC3);
  tmpImg -= meanVal;

  cv::Mat paddedImg(paddedH, paddedW, CV_32FC3, cv::Scalar(0, 0, 0));
  tmpImg.copyTo(paddedImg(cv::Rect(0, 0, newW, newH)));

  this->preprocess(dst, paddedImg, paddedW, paddedH, 3);

  // boxes, labels, scores, masks
  auto inferenceOutput = (*this)({dst});

  assert(inferenceOutput[1].second.size() == 1);
  size_t nBoxes = inferenceOutput[1].second[0];

  std::vector<std::array<float, 4>> bboxes;
  std::vector<uint64_t> classIndices;
  std::vector<float> scores;
  std::vector<cv::Mat> masks;

  bboxes.reserve(nBoxes);
  classIndices.reserve(nBoxes);
  scores.reserve(nBoxes);
  masks.reserve(nBoxes);


  for (size_t i = 0; i < nBoxes; ++i) {
    if (inferenceOutput[2].first[i] > confThresh) {
      float xmin = inferenceOutput[0].first[i * 4 + 0] / ratio;
      float ymin = inferenceOutput[0].first[i * 4 + 1] / ratio;
      float xmax = inferenceOutput[0].first[i * 4 + 2] / ratio;
      float ymax = inferenceOutput[0].first[i * 4 + 3] / ratio;

      xmin = std::max<float>(xmin, 0);
      ymin = std::max<float>(ymin, 0);
      xmax = std::min<float>(xmax, inputImg.cols);
      ymax = std::min<float>(ymax, inputImg.rows);

      bboxes.emplace_back(std::array<float, 4>{xmin, ymin, xmax, ymax});
      classIndices.emplace_back(reinterpret_cast<int64_t *>(inferenceOutput[1].first)[i]);
      scores.emplace_back(inferenceOutput[2].first[i]);

      cv::Mat curMask(28, 28, CV_32FC1);
      memcpy(
        curMask.data,
        inferenceOutput[3].first + i * 28 * 28,
        28 * 28 * sizeof(float));
      masks.emplace_back(curMask);
    }
  }

  if (bboxes.size() == 0) {
    return inputImg;
  }

  EPD::activateUseCase(inputImg, bboxes, classIndices, scores, masks, this->getClassNames());
  return visualize(inputImg, bboxes, classIndices, masks, this->getClassNames(), 0.5);
}

// Mutator 5: Localization
cv::Mat P3OrtBase::infer_visualize(
  const cv::Mat & inputImg,
  const cv::Mat & depthImg,
  sensor_msgs::msg::CameraInfo camera_info,
  int newW,
  int newH,
  int paddedW,
  int paddedH,
  float ratio,
  float * dst,
  float confThresh,
  const cv::Scalar & meanVal)
{
  cv::Mat tmpImg;
  cv::resize(inputImg, tmpImg, cv::Size(newW, newH));

  tmpImg.convertTo(tmpImg, CV_32FC3);
  tmpImg -= meanVal;

  cv::Mat paddedImg(paddedH, paddedW, CV_32FC3, cv::Scalar(0, 0, 0));
  tmpImg.copyTo(paddedImg(cv::Rect(0, 0, newW, newH)));

  this->preprocess(dst, paddedImg, paddedW, paddedH, 3);

  // boxes, labels, scores, masks
  auto inferenceOutput = (*this)({dst});

  assert(inferenceOutput[1].second.size() == 1);
  size_t nBoxes = inferenceOutput[1].second[0];

  std::vector<std::array<float, 4>> bboxes;
  std::vector<uint64_t> classIndices;
  std::vector<float> scores;
  std::vector<cv::Mat> masks;

  bboxes.reserve(nBoxes);
  classIndices.reserve(nBoxes);
  scores.reserve(nBoxes);
  masks.reserve(nBoxes);


  for (size_t i = 0; i < nBoxes; ++i) {
    if (inferenceOutput[2].first[i] > confThresh) {
      float xmin = inferenceOutput[0].first[i * 4 + 0] / ratio;
      float ymin = inferenceOutput[0].first[i * 4 + 1] / ratio;
      float xmax = inferenceOutput[0].first[i * 4 + 2] / ratio;
      float ymax = inferenceOutput[0].first[i * 4 + 3] / ratio;

      xmin = std::max<float>(xmin, 0);
      ymin = std::max<float>(ymin, 0);
      xmax = std::min<float>(xmax, inputImg.cols);
      ymax = std::min<float>(ymax, inputImg.rows);

      bboxes.emplace_back(std::array<float, 4>{xmin, ymin, xmax, ymax});
      classIndices.emplace_back(reinterpret_cast<int64_t *>(inferenceOutput[1].first)[i]);
      scores.emplace_back(inferenceOutput[2].first[i]);

      cv::Mat curMask(28, 28, CV_32FC1);
      memcpy(
        curMask.data,
        inferenceOutput[3].first + i * 28 * 28,
        28 * 28 * sizeof(float));
      masks.emplace_back(curMask);
    }
  }

  if (bboxes.size() == 0) {
    return inputImg;
  }

  return localize_visualize(
    inputImg,
    depthImg,
    camera_info,
    bboxes,
    classIndices,
    masks,
    this->getClassNames(),
    0.5);
}

// Mutator 4
EPD::EPDObjectDetection P3OrtBase::infer_action(
  const cv::Mat & inputImg,
  int newW,
  int newH,
  int paddedW,
  int paddedH,
  float ratio,
  float * dst,
  float confThresh,
  const cv::Scalar & meanVal)
{
  cv::Mat tmpImg;
  cv::resize(inputImg, tmpImg, cv::Size(newW, newH));

  tmpImg.convertTo(tmpImg, CV_32FC3);
  tmpImg -= meanVal;

  cv::Mat paddedImg(paddedH, paddedW, CV_32FC3, cv::Scalar(0, 0, 0));
  tmpImg.copyTo(paddedImg(cv::Rect(0, 0, newW, newH)));

  this->preprocess(dst, paddedImg, paddedW, paddedH, 3);

  // boxes, labels, scores, masks
  auto inferenceOutput = (*this)({dst});

  assert(inferenceOutput[1].second.size() == 1);
  size_t nBoxes = inferenceOutput[1].second[0];

  std::vector<std::array<float, 4>> bboxes;
  std::vector<uint64_t> classIndices;
  std::vector<float> scores;
  std::vector<cv::Mat> masks;

  bboxes.reserve(nBoxes);
  classIndices.reserve(nBoxes);
  scores.reserve(nBoxes);
  masks.reserve(nBoxes);

  for (size_t i = 0; i < nBoxes; ++i) {
    if (inferenceOutput[2].first[i] > confThresh) {
      float xmin = inferenceOutput[0].first[i * 4 + 0] / ratio;
      float ymin = inferenceOutput[0].first[i * 4 + 1] / ratio;
      float xmax = inferenceOutput[0].first[i * 4 + 2] / ratio;
      float ymax = inferenceOutput[0].first[i * 4 + 3] / ratio;

      xmin = std::max<float>(xmin, 0);
      ymin = std::max<float>(ymin, 0);
      xmax = std::min<float>(xmax, inputImg.cols);
      ymax = std::min<float>(ymax, inputImg.rows);

      bboxes.emplace_back(std::array<float, 4>{xmin, ymin, xmax, ymax});
      classIndices.emplace_back(reinterpret_cast<int64_t *>(inferenceOutput[1].first)[i]);
      scores.emplace_back(inferenceOutput[2].first[i]);

      cv::Mat curMask(28, 28, CV_32FC1);
      memcpy(
        curMask.data,
        inferenceOutput[3].first + i * 28 * 28,
        28 * 28 * sizeof(float));
      masks.emplace_back(curMask);
    }
  }

  if (bboxes.size() == 0) {
    EPD::EPDObjectDetection output_msg(0);
    return output_msg;
  }

  EPD::activateUseCase(inputImg, bboxes, classIndices, scores, masks, this->getClassNames());

  EPD::EPDObjectDetection output_obj(bboxes.size());
  output_obj.bboxes = bboxes;
  output_obj.classIndices = classIndices;
  output_obj.scores = scores;
  output_obj.masks = masks;

  return output_obj;
}

cv::Mat P3OrtBase::visualize(
  const cv::Mat & img,
  const std::vector<std::array<float, 4>> & bboxes,
  const std::vector<uint64_t> & classIndices,
  const std::vector<cv::Mat> & masks,
  const std::vector<std::string> & allClassNames = {},
  const float maskThreshold = 0.5)
{
  assert(bboxes.size() == classIndices.size());
  if (!allClassNames.empty()) {
    assert(allClassNames.size() > *std::max_element(classIndices.begin(), classIndices.end()));
  }

  cv::Scalar allColors(255.0, 0.0, 0.0, 0.0);

  cv::Mat result = img.clone();

  for (size_t i = 0; i < bboxes.size(); ++i) {
    const auto & curBbox = bboxes[i];
    const uint64_t classIdx = classIndices[i];
    cv::Mat curMask = masks[i].clone();
    const cv::Scalar & curColor = allColors;
    const std::string curLabel = allClassNames.empty() ?
      std::to_string(classIdx) : allClassNames[classIdx];

    cv::rectangle(
      result, cv::Point(curBbox[0], curBbox[1]),
      cv::Point(curBbox[2], curBbox[3]), curColor, 2);

    int baseLine = 0;
    cv::Size labelSize = cv::getTextSize(
      curLabel, cv::FONT_HERSHEY_COMPLEX,
      0.35, 1, &baseLine);
    cv::rectangle(
      result, cv::Point(curBbox[0], curBbox[1]),
      cv::Point(
        curBbox[0] + labelSize.width, curBbox[1] +
        static_cast<int>(1.3 * labelSize.height)),
      curColor, -1);
    cv::putText(
      result, curLabel, cv::Point(curBbox[0], curBbox[1] + labelSize.height),
      cv::FONT_HERSHEY_COMPLEX,
      0.35, cv::Scalar(255, 255, 255));

    // Visualize masks
    const cv::Rect curBoxRect(cv::Point(curBbox[0], curBbox[1]),
      cv::Point(curBbox[2], curBbox[3]));

    cv::resize(curMask, curMask, curBoxRect.size());

    cv::Mat finalMask = (curMask > maskThreshold);

    cv::Mat coloredRoi = (0.3 * curColor + 0.7 * result(curBoxRect));

    coloredRoi.convertTo(coloredRoi, CV_8UC3);

    std::vector<cv::Mat> contours;
    cv::Mat hierarchy;
    finalMask.convertTo(finalMask, CV_8U);

    cv::findContours(finalMask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(coloredRoi, contours, -1, curColor, 5, cv::LINE_8, hierarchy, 100);
    coloredRoi.copyTo(result(curBoxRect), finalMask);
  }
  return result;
}

double P3OrtBase::findMedian(cv::Mat depthImg)
{
  double m = (depthImg.rows * depthImg.cols) / 2;
  int bin = 0;
  double median = -1.0;

  // Setting to hardcoded 2000 millimeters
  // This is the limit of intel realsense D415.
  int histSize = 2000;
  float range[] = {0, 2000};
  const float * histRange = {range};
  bool uniform = true;
  bool accumulate = false;
  cv::Mat hist;
  cv::calcHist(&depthImg, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

  for (int i = 0; i < histSize && median < 0.0; ++i) {
    bin += cvRound(hist.at<float>(i));
    if (bin > m && median < 0.0) {
      median = i;
    }
  }

  return median;
}

cv::Mat P3OrtBase::localize_visualize(
  const cv::Mat & img,
  const cv::Mat & depthImg,
  sensor_msgs::msg::CameraInfo camera_info,
  const std::vector<std::array<float, 4>> & bboxes,
  const std::vector<uint64_t> & classIndices,
  const std::vector<cv::Mat> & masks,
  const std::vector<std::string> & allClassNames = {},
  const float maskThreshold = 0.5)
{
  assert(bboxes.size() == classIndices.size());
  if (!allClassNames.empty()) {
    assert(
      allClassNames.size() >
      *std::max_element(classIndices.begin(), classIndices.end()));
  }

  cv::Scalar allColors(0.0, 0.0, 255.0, 0.0);

  // // num of objects will be equal to number of bboxes
  cv::Mat result = img.clone();
  for (size_t i = 0; i < bboxes.size(); ++i) {
    const auto & curBbox = bboxes[i];
    const uint64_t classIdx = classIndices[i];
    cv::Mat curMask = masks[i].clone();
    const cv::Scalar & curColor = allColors[classIdx];
    const std::string curLabel = allClassNames.empty() ?
      std::to_string(classIdx) :
      allClassNames[classIdx];

    cv::rectangle(
      result, cv::Point(curBbox[0], curBbox[1]),
      cv::Point(curBbox[2], curBbox[3]), curColor, 2);

    int baseLine = 0;
    cv::Size labelSize =
      cv::getTextSize(curLabel, cv::FONT_HERSHEY_COMPLEX, 0.35, 1, &baseLine);
    cv::rectangle(
      result, cv::Point(
        curBbox[0], curBbox[1]),
      cv::Point(
        curBbox[0] + labelSize.width,
        curBbox[1] + static_cast<int>(1.3 * labelSize.height)),
      curColor, -1);
    cv::putText(
      result, curLabel,
      cv::Point(curBbox[0], curBbox[1] + labelSize.height),
      cv::FONT_HERSHEY_COMPLEX, 0.35, cv::Scalar(255, 255, 255));

    // Visualizing masks
    const cv::Rect curBoxRect(cv::Point(curBbox[0], curBbox[1]),
      cv::Point(curBbox[2], curBbox[3]));

    cv::resize(curMask, curMask, curBoxRect.size());

    // Assigning masks that exceed the maskThreshold.
    cv::Mat finalMask = (curMask > maskThreshold);

    // Assigning coloredRoi with the bounding box.
    cv::Mat coloredRoi = (0.3 * curColor + 0.7 * result(curBoxRect));

    coloredRoi.convertTo(coloredRoi, CV_8UC3);

    std::vector<cv::Mat> contours;
    cv::Mat hierarchy;
    cv::Mat tempFinalMask;
    finalMask.convertTo(tempFinalMask, CV_8U);
    // Generate and draw contours on output image.
    // Contours will be used later.
    cv::findContours(
      tempFinalMask, contours, hierarchy, cv::RETR_TREE,
      cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(
      coloredRoi, contours, -1, cv::Scalar(0, 0, 255), 5, cv::LINE_8,
      hierarchy, 100);

    // For more details, refer to link below:
    // https://tinyurl.com/y5qnnxud
    float ppx = camera_info.k.at(2);
    float fx = camera_info.k.at(0);
    float ppy = camera_info.k.at(5);
    float fy = camera_info.k.at(4);

    // Getting rotated rectangle and draw the major axis
    std::vector<cv::RotatedRect> minRect(contours.size());
    std::vector<float> angles;
    float angle;
    float obj_surface_depth;
    float object_length, object_breadth, object_height;
    cv::Point pt_a, pt_b, pt_c, pt_d;
    cv::Point rotated_mid;

    // Getting only the largest contour
    // The largest contour is the one which has the largest area.
    // TODO(cardboardcode): Changed according to your use case.
    double maxArea = 0;
    int maxAreaContourId = 999;
    for (unsigned int j = 0; j < contours.size(); j++) {
      double newArea = cv::contourArea(contours[j]);
      if (newArea > maxArea) {
        maxArea = newArea;
        maxAreaContourId = j;
      }  //  End if
    }  //  End for
    unsigned int maxID = maxAreaContourId;

    for (unsigned int index = 0; index < contours.size(); index++) {
      if (index != maxID) {
        continue;
      }
      //  Compute rotated rectangle based on contours
      minRect[index] = cv::minAreaRect(cv::Mat(contours[index]));
      cv::Point2f rect_points[4];
      //  4 points of the rotated rectangle
      minRect[index].points(rect_points);

      //  Mid points of the each side of the rotated rectangle
      pt_a = (rect_points[0] + rect_points[3]) / 2;
      pt_b = (rect_points[1] + rect_points[2]) / 2;
      pt_c = (rect_points[0] + rect_points[1]) / 2;
      pt_d = (rect_points[3] + rect_points[2]) / 2;

      //  Add the top left corner to the coordinate in the small bbox
      //  For temporary, bboxes center
      rotated_mid = (cv::Point(curBbox[0], curBbox[1]) +
        cv::Point(curBbox[2], curBbox[3])) / 2;

      //  Get coordinates of the object center
      float table_depth = this->findMedian(depthImg) * 0.001;
      obj_surface_depth = this->findMedian(depthImg(curBoxRect)) * 0.001;

      float x = (rotated_mid.x - ppx) / fx * obj_surface_depth;
      float y = (rotated_mid.y - ppy) / fy * obj_surface_depth;

      int x_axis_offset = -0.017;
      int y_axis_offset = -0.01;

      std::cout << "[-OBJ centroid x-] = " << x + x_axis_offset << std::endl;
      std::cout << "[-OBJ centroid y-] = " << y + y_axis_offset << std::endl;
      std::cout << "[-OBJ centroid z-] = " << obj_surface_depth +
      (table_depth - obj_surface_depth) / 2 <<
        std::endl;

      // Get Real Size and angle of object
      // Compare the length of 2 side of the rectangle,
      // the longer side will be the major axis
      if (cv::norm(rect_points[0] - rect_points[1]) >
        cv::norm(rect_points[1] - rect_points[2]))
      {
        // Draws the major axis(red)
        cv::line(coloredRoi, pt_a, pt_b, cv::Scalar(0, 0, 255), 2);
        // Draws the minor axis (green)
        cv::line(coloredRoi, pt_c, pt_d, cv::Scalar(0, 255, 0), 2);
        // Calculates the angle
        angle = round(atan2(pt_a.y - pt_b.y, pt_a.x - pt_b.x) * 100) / 100;
        // Calculates the length of the object
        object_length = obj_surface_depth * sqrt(
          pow((pt_a.x - pt_b.x) / fx, 2) +
          pow(
            (pt_a.y - pt_b.y) / fy,
            2));
        // Calculates the breadth of the object
        object_breadth = obj_surface_depth * sqrt(
          pow((pt_c.x - pt_d.x) / fx, 2) +
          pow(
            (pt_c.y - pt_d.y) / fy,
            2));

      } else {
        // Draw the major axis
        cv::line(coloredRoi, pt_c, pt_d, cv::Scalar(0, 0, 255), 2);
        // Draw the minor axis (green)
        cv::line(coloredRoi, pt_a, pt_b, cv::Scalar(0, 255, 0), 2);
        // Calculate the angle
        angle = round(atan2(pt_c.y - pt_d.y, pt_c.x - pt_d.x) * 100) / 100;
        // Get object breadth and length
        object_breadth = obj_surface_depth * sqrt(
          pow((pt_a.x - pt_b.x) / fx, 2) +
          pow((pt_a.y - pt_b.y) / fy, 2));
        object_length = obj_surface_depth * sqrt(
          pow((pt_c.x - pt_d.x) / fx, 2) +
          pow((pt_c.y - pt_d.y) / fy, 2));
      }
      // Setting height of object
      object_height = table_depth - obj_surface_depth;

      // TODO(cardboardcode): To provide optimized debug prints in future iterations.
      std::cout << "[-OBJ Name-] = " << curLabel << std::endl;
      std::cout << "[-OBJ Length-] = " << object_length << std::endl;
      std::cout << "[-OBJ Breadth-] = " << object_breadth << std::endl;
      std::cout << "[-OBJ Height-] = " << object_height << std::endl;

      // mark the center point with blue dot
      cv::circle(coloredRoi, rotated_mid, 1, cv::Scalar(255, 0, 0), 1);

      angle = angle * 180 / CV_PI;
      // Make angle value absolute for anti-clockwise angle
      if (angle > 0) {
        angle -= 180;
        angle *= -1;
      }

      cv::putText(
        result, cv::format("%.2f", angle),
        cv::Point(curBbox[0], curBbox[1] - labelSize.height),
        cv::FONT_HERSHEY_COMPLEX, 0.75,
        cv::Scalar(0, 0, 255));
    }

    // Push each Region-Of-Interest (ROI) in sequence
    coloredRoi.copyTo(result(curBoxRect), finalMask);
  }

  return result;
}

// DEBUG
// A mutator function that will output an EPD::EPDObjectLocalization object that
// contains all information required for Localization.
EPD::EPDObjectLocalization P3OrtBase::infer_action(
  const cv::Mat & inputImg,
  const cv::Mat & depthImg,
  sensor_msgs::msg::CameraInfo camera_info,
  int newW,
  int newH,
  int paddedW,
  int paddedH,
  float ratio,
  float * dst,
  float confThresh,
  const cv::Scalar & meanVal)
{
  cv::Mat tmpImg;
  cv::resize(inputImg, tmpImg, cv::Size(newW, newH));

  tmpImg.convertTo(tmpImg, CV_32FC3);
  tmpImg -= meanVal;

  cv::Mat paddedImg(paddedH, paddedW, CV_32FC3, cv::Scalar(0, 0, 0));
  tmpImg.copyTo(paddedImg(cv::Rect(0, 0, newW, newH)));

  this->preprocess(dst, paddedImg, paddedW, paddedH, 3);

  // boxes, labels, scores, masks
  auto inferenceOutput = (*this)({dst});

  assert(inferenceOutput[1].second.size() == 1);
  size_t nBoxes = inferenceOutput[1].second[0];

  std::vector<std::array<float, 4>> bboxes;
  std::vector<uint64_t> classIndices;
  std::vector<float> scores;
  std::vector<cv::Mat> masks;

  bboxes.reserve(nBoxes);
  classIndices.reserve(nBoxes);
  scores.reserve(nBoxes);
  masks.reserve(nBoxes);

  for (size_t i = 0; i < nBoxes; ++i) {
    if (inferenceOutput[2].first[i] > confThresh) {
      float xmin = inferenceOutput[0].first[i * 4 + 0] / ratio;
      float ymin = inferenceOutput[0].first[i * 4 + 1] / ratio;
      float xmax = inferenceOutput[0].first[i * 4 + 2] / ratio;
      float ymax = inferenceOutput[0].first[i * 4 + 3] / ratio;

      xmin = std::max<float>(xmin, 0);
      ymin = std::max<float>(ymin, 0);
      xmax = std::min<float>(xmax, inputImg.cols);
      ymax = std::min<float>(ymax, inputImg.rows);

      bboxes.emplace_back(std::array<float, 4>{xmin, ymin, xmax, ymax});
      classIndices.emplace_back(reinterpret_cast<int64_t *>(inferenceOutput[1].first)[i]);
      scores.emplace_back(inferenceOutput[2].first[i]);

      cv::Mat curMask(28, 28, CV_32FC1);
      memcpy(
        curMask.data,
        inferenceOutput[3].first + i * 28 * 28,
        28 * 28 * sizeof(float));
      masks.emplace_back(curMask);
    }
  }

  std::vector<std::string> allClassNames = this->getClassNames();
  float maskThreshold = 0.5;
  cv::Scalar allColors(0.0, 0.0, 255.0, 0.0);
  cv::Mat result = inputImg.clone();

  assert(bboxes.size() == classIndices.size());
  if (!allClassNames.empty()) {
    assert(
      allClassNames.size() >
      *std::max_element(classIndices.begin(), classIndices.end()));
  }

  // If there is zero bounding boxes generated, return empty EPDObjectLocalization object.
  if (bboxes.size() == 0) {
    EPD::EPDObjectLocalization output_msg(0);
    return output_msg;
  }

  EPD::EPDObjectLocalization output_obj(bboxes.size());

  float table_depth = this->findMedian(depthImg) * 0.001;

  // No. of objects will be equal to number of bboxes
  /* START of Populating EPDObjectLocalization object */
  for (size_t i = 0; i < bboxes.size(); ++i) {
    const auto & curBbox = bboxes[i];
    const uint64_t classIdx = classIndices[i];
    cv::Mat curMask = masks[i].clone();
    const std::string curLabel = allClassNames.empty() ?
      std::to_string(classIdx) :
      allClassNames[classIdx];

    output_obj.objects[i].name = curLabel;
    // Top x of ROI
    output_obj.objects[i].roi.x_offset = curBbox[0];
    // Top y of ROI
    output_obj.objects[i].roi.y_offset = curBbox[1];
    // Bounding Box height as ROI
    output_obj.objects[i].roi.height = curBbox[3] - curBbox[1];
    // Bounding Box width as ROI
    output_obj.objects[i].roi.width = curBbox[2] - curBbox[0];

    // Visualizing masks
    const cv::Rect curBoxRect(cv::Point(curBbox[0], curBbox[1]),
      cv::Point(curBbox[2], curBbox[3]));

    cv::resize(curMask, curMask, curBoxRect.size());

    // Assigning masks that exceed the maskThreshold.
    cv::Mat finalMask = (curMask > maskThreshold);

    std::vector<cv::Mat> contours;
    cv::Mat hierarchy;
    cv::Mat tempFinalMask;
    finalMask.convertTo(tempFinalMask, CV_8U);
    // Generate and draw contours on output image.
    // Contours will be used later.
    cv::findContours(
      tempFinalMask, contours, hierarchy, cv::RETR_TREE,
      cv::CHAIN_APPROX_SIMPLE);

    // For more details, refer to link below:
    // https://tinyurl.com/y5qnnxud
    float ppx = camera_info.k.at(2);
    float fx = camera_info.k.at(0);
    float ppy = camera_info.k.at(5);
    float fy = camera_info.k.at(4);

    // Getting rotated rectangle and draw the major axis
    std::vector<cv::RotatedRect> minRect(contours.size());
    std::vector<float> angles;
    float angle, obj_surface_depth;
    cv::Point pt_a, pt_b, pt_c, pt_d;
    cv::Point rotated_mid;

    // Getting only the largest contour
    // The largest contour is the one which has the largest area.
    double maxArea = 0;
    int maxAreaContourId = 999;
    for (unsigned int j = 0; j < contours.size(); j++) {
      double newArea = cv::contourArea(contours[j]);
      if (newArea > maxArea) {
        maxArea = newArea;
        maxAreaContourId = j;
      }  // End if
    }  // End for
    unsigned int maxID = maxAreaContourId;

    for (unsigned int index = 0; index < contours.size(); index++) {
      if (index != maxID) {
        continue;
      }
      // Function that compute rotated rectangle based on contours
      minRect[index] = cv::minAreaRect(cv::Mat(contours[index]));
      cv::Point2f rect_points[4];
      // 4 points of the rotated rectangle
      minRect[index].points(rect_points);

      // Mid points of the each side of the rotated rectangle
      pt_a = (rect_points[0] + rect_points[3]) / 2;
      pt_b = (rect_points[1] + rect_points[2]) / 2;
      pt_c = (rect_points[0] + rect_points[1]) / 2;
      pt_d = (rect_points[3] + rect_points[2]) / 2;

      // For temporary, bboxes center
      rotated_mid = (cv::Point(curBbox[0], curBbox[1]) +
        cv::Point(curBbox[2], curBbox[3])) / 2;

      obj_surface_depth = this->findMedian(depthImg(curBoxRect)) * 0.001;
      float x = (rotated_mid.x - ppx) / fx * obj_surface_depth;
      float y = (rotated_mid.y - ppy) / fy * obj_surface_depth;

      int x_axis_offset = -0.017;
      int y_axis_offset = -0.01;
      output_obj.objects[i].pos.pose.position.x = x + x_axis_offset;
      output_obj.objects[i].pos.pose.position.y = y + y_axis_offset;
      output_obj.objects[i].pos.pose.position.z = obj_surface_depth +
        (table_depth - obj_surface_depth) / 2;

      // Get Real Size and angle of object
      // Compare the length of 2 side of the rectangle,
      // the longer side will be the major axis
      if (cv::norm(rect_points[0] - rect_points[1]) >
        cv::norm(rect_points[1] - rect_points[2]))
      {
        // Calculates the angle
        angle = round(atan2(pt_a.y - pt_b.y, pt_a.x - pt_b.x) * 100) / 100;
        // Calculates the length of the object
        output_obj.objects[i].length = obj_surface_depth * sqrt(
          pow((pt_a.x - pt_b.x) / fx, 2) +
          pow((pt_a.y - pt_b.y) / fy, 2));
        // Calculates the breadth of the object
        output_obj.objects[i].breadth = obj_surface_depth * sqrt(
          pow((pt_c.x - pt_d.x) / fx, 2) +
          pow((pt_c.y - pt_d.y) / fy, 2));
      } else {
        // Calculates the angle
        angle = round(atan2(pt_c.y - pt_d.y, pt_c.x - pt_d.x) * 100) / 100;
        // Gets object breadth and length
        output_obj.objects[i].breadth = obj_surface_depth * sqrt(
          pow((pt_a.x - pt_b.x) / fx, 2) +
          pow((pt_a.y - pt_b.y) / fy, 2));
        output_obj.objects[i].length = obj_surface_depth * sqrt(
          pow((pt_c.x - pt_d.x) / fx, 2) +
          pow((pt_c.y - pt_d.y) / fy, 2));
      }
      // Setting height of object
      output_obj.objects[i].height = table_depth - obj_surface_depth;

      angle = angle * 180 / CV_PI;
      // Make angle value absolute for anti-clockwise angle
      if (angle > 0) {
        angle -= 180;
        angle *= -1;
      }

      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, angle * CV_PI / 180);
      output_obj.objects[i].pos.pose.orientation.x = myQuaternion[0];
      output_obj.objects[i].pos.pose.orientation.y = myQuaternion[1];
      output_obj.objects[i].pos.pose.orientation.z = myQuaternion[2];
      output_obj.objects[i].pos.pose.orientation.w = myQuaternion[3];

      // Populate Header in pos variable of custom_msgs object.
      output_obj.objects[i].pos.header = std_msgs::msg::Header();
      output_obj.objects[i].pos.header.frame_id = "detected_obj_" + std::to_string(i);
    }
  }
  // END of Populating EPDObjectLocalization object
  return output_obj;
}

}  // namespace Ort
