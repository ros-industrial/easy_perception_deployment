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

#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/centroid.h"
#include "pcl/common/eigen.h"

#include "opencv2/opencv.hpp"

#include "tf2/LinearMath/Quaternion.h"

#include "p3_ort_base.hpp"
#include "epd_utils_lib/usecase_config.hpp"


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

// Mutator: Localization
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

// Mutator: Tracking
cv::Mat P3OrtBase::infer_visualize(
  const cv::Mat & inputImg,
  const cv::Mat & depthImg,
  sensor_msgs::msg::CameraInfo camera_info,
  const std::string tracker_type,
  std::vector<cv::Ptr<cv::Tracker>> & trackers,
  std::vector<int> & tracker_logs,
  std::vector<EPD::LabelledRect2d> & tracker_results)
{
  std::vector<float> dst(3 * m_paddedH * m_paddedW);

  return this->infer_visualize(
    inputImg, depthImg, camera_info, tracker_type, trackers, tracker_logs,
    tracker_results, m_newW, m_newH, m_paddedW, m_paddedH, m_ratio,
    dst.data(), 0.5, cv::Scalar(102.9801, 115.9465, 122.7717));
}

// Mutator: Classification
EPD::EPDObjectDetection P3OrtBase::infer(const cv::Mat & inputImg)
{
  std::vector<float> dst(3 * m_paddedH * m_paddedW);

  return this->infer(
    inputImg, m_newW, m_newH,
    m_paddedW, m_paddedH, m_ratio, dst.data(), 0.5,
    cv::Scalar(102.9801, 115.9465, 122.7717));
}

// Mutator: Localization
EPD::EPDObjectLocalization P3OrtBase::infer_action(
  const cv::Mat & inputImg,
  const cv::Mat & depthImg,
  sensor_msgs::msg::CameraInfo camera_info,
  double camera_to_plane_distance_mm)
{
  std::vector<float> dst(3 * m_paddedH * m_paddedW);

  return this->infer_action(
    inputImg, depthImg, camera_info, camera_to_plane_distance_mm,
    m_newW, m_newH, m_paddedW, m_paddedH, m_ratio, dst.data(), 0.5,
    cv::Scalar(102.9801, 115.9465, 122.7717));
}

// Mutator: Tracking
EPD::EPDObjectTracking P3OrtBase::infer_action(
  const cv::Mat & inputImg,
  const cv::Mat & depthImg,
  sensor_msgs::msg::CameraInfo camera_info,
  double camera_to_plane_distance_mm,
  const std::string tracker_type,
  std::vector<cv::Ptr<cv::Tracker>> & trackers,
  std::vector<int> & tracker_logs,
  std::vector<EPD::LabelledRect2d> & tracker_results)
{
  std::vector<float> dst(3 * m_paddedH * m_paddedW);

  return this->infer_action(
    inputImg, depthImg, camera_info, camera_to_plane_distance_mm,
    tracker_type, trackers, tracker_logs, tracker_results,
    m_newW, m_newH, m_paddedW, m_paddedH, m_ratio, dst.data(), 0.5,
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

// Mutator 5: Tracking
cv::Mat P3OrtBase::infer_visualize(
  const cv::Mat & inputImg,
  const cv::Mat & depthImg,
  sensor_msgs::msg::CameraInfo camera_info,
  const std::string tracker_type,
  std::vector<cv::Ptr<cv::Tracker>> & trackers,
  std::vector<int> & tracker_logs,
  std::vector<EPD::LabelledRect2d> & tracker_results,
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
  // DEBUG
  // Evaluate detection results and tracking results.
  tracking_evaluate(bboxes, inputImg, tracker_type, trackers, tracker_logs, tracker_results);

  if (bboxes.size() == 0) {
    return inputImg;
  }

  return tracking_visualize(
    inputImg,
    depthImg,
    camera_info,
    tracker_results,
    bboxes,
    classIndices,
    masks,
    this->getClassNames(),
    0.5);
}

// Mutator 4
EPD::EPDObjectDetection P3OrtBase::infer(
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

  std::vector<std::array<int, 4>> bboxes;
  std::vector<uint64_t> classIndices;
  std::vector<float> scores;
  std::vector<cv::Mat> masks;

  bboxes.reserve(nBoxes);
  classIndices.reserve(nBoxes);
  scores.reserve(nBoxes);
  masks.reserve(nBoxes);

  for (size_t i = 0; i < nBoxes; ++i) {
    if (inferenceOutput[2].first[i] > confThresh) {
      int xmin = inferenceOutput[0].first[i * 4 + 0] / ratio;
      int ymin = inferenceOutput[0].first[i * 4 + 1] / ratio;
      int xmax = inferenceOutput[0].first[i * 4 + 2] / ratio;
      int ymax = inferenceOutput[0].first[i * 4 + 3] / ratio;

      xmin = std::max<int>(xmin, 0);
      ymin = std::max<int>(ymin, 0);
      xmax = std::min<int>(xmax, inputImg.cols);
      ymax = std::min<int>(ymax, inputImg.rows);

      bboxes.emplace_back(std::array<int, 4>{xmin, ymin, xmax, ymax});
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

  EPD::EPDObjectDetection output_obj(bboxes.size());
  output_obj.bboxes = bboxes;
  output_obj.classIndices = classIndices;
  output_obj.scores = scores;
  output_obj.masks = masks;

  return output_obj;
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

double P3OrtBase::findMin(cv::Mat depthImg)
{
  int bin = 0;
  double min = -1.0;

  // Setting to hardcoded 2000 millimeters
  // This is the limit of intel realsense D415.
  int histSize = 2000;
  float range[] = {0, 2000};
  const float * histRange = {range};
  bool uniform = true;
  bool accumulate = false;
  cv::Mat hist;
  cv::calcHist(&depthImg, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

  for (int i = 0; i < histSize; ++i) {
    bin += cvRound(hist.at<float>(i));
    // Store the first depth value that is shared among more than 1 point.
    // Break and escape for loop.
    if (i != 0 && cvRound(hist.at<float>(i)) > 0) {
      // std::cout << "Depth Value = " << i << " has " << cvRound(hist.at<float>(i)) << std::endl;
      min = i;
      break;
    }
  }

  return min;
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
    // Generate red contour lines of segmentation mask.
    cv::findContours(
      tempFinalMask, contours, hierarchy, cv::RETR_TREE,
      cv::CHAIN_APPROX_SIMPLE);
    // Draw red contour lines on output image.
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
      obj_surface_depth = this->findMin(depthImg(curBoxRect)) * 0.001;

      float x = (rotated_mid.x - ppx) / fx * obj_surface_depth;
      float y = (rotated_mid.y - ppy) / fy * obj_surface_depth;

      std::cout << "[-cam -> table-] = " << table_depth <<
        " meters" << std::endl;
      std::cout << "[-cam -> obj_top-] = " << obj_surface_depth <<
        " meters" << std::endl;

      std::cout << "[-OBJ centroid x-] = " << x << std::endl;
      std::cout << "[-OBJ centroid y-] = " << y << std::endl;
      std::cout << "[-OBJ centroid z-] = " << obj_surface_depth +
      (table_depth - obj_surface_depth) / 2 << std::endl;

      // Get estimated size of object
      // Compare the length of 2 side of the rectangle,
      // the longer side will be the major axis
      if (cv::norm(rect_points[0] - rect_points[1]) >
        cv::norm(rect_points[1] - rect_points[2]))
      {
        // Draws the major axis(red)
        cv::line(coloredRoi, pt_a, pt_b, cv::Scalar(0, 0, 255), 2);
        // Draws the minor axis (green)
        cv::line(coloredRoi, pt_c, pt_d, cv::Scalar(0, 255, 0), 2);
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
      std::cout << "[-OBJ Length-] = " << object_length << " meters" << std::endl;
      std::cout << "[-OBJ Breadth-] = " << object_breadth << " meters" << std::endl;
      std::cout << "[-OBJ Height-] = " << object_height << " meters" << std::endl;

      // Mark the center point with blue dot
      cv::circle(coloredRoi, rotated_mid, 1, cv::Scalar(255, 0, 0), 1);
    }

    // Push each Region-Of-Interest (ROI) in sequence
    coloredRoi.copyTo(result(curBoxRect), finalMask);
  }

  return result;
}

cv::Mat P3OrtBase::tracking_visualize(
  const cv::Mat & img,
  const cv::Mat & depthImg,
  sensor_msgs::msg::CameraInfo camera_info,
  std::vector<EPD::LabelledRect2d> & tracker_results,
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
  cv::Scalar trackingColor(255.0, 0.0, 0.0, 0.0);

  cv::Mat result = img.clone();

  for (size_t i = 0; i < tracker_results.size(); i++) {
    cv::rectangle(
      result,
      tracker_results[i].obj_bounding_box,
      trackingColor,
      4);
  }

  for (size_t i = 0; i < bboxes.size(); ++i) {
    const auto & curBbox = bboxes[i];
    const uint64_t classIdx = classIndices[i];
    cv::Mat curMask = masks[i].clone();
    const cv::Scalar & curColor = allColors[classIdx];
    std::string curLabel = allClassNames.empty() ?
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

    // Update curLabel with tracker number tag for object.
    for (size_t j = 0; j < tracker_results.size(); j++) {
      if (tracker_results[j].obj_bounding_box.x == curBbox[0] &&
        tracker_results[j].obj_bounding_box.y == curBbox[1] &&
        tracker_results[j].obj_bounding_box.width == curBbox[2] - curBbox[0] &&
        tracker_results[j].obj_bounding_box.height == curBbox[3] - curBbox[1])
      {
        curLabel = curLabel + "_" + std::string(tracker_results[j].obj_tag);
      }
    }
    cv::putText(
      result, curLabel,
      cv::Point(curBbox[0], curBbox[1] + labelSize.height),
      cv::FONT_HERSHEY_COMPLEX, 0.55, cv::Scalar(255, 255, 255));

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
    // Generate red contour lines of segmentation mask.
    cv::findContours(
      tempFinalMask, contours, hierarchy, cv::RETR_TREE,
      cv::CHAIN_APPROX_SIMPLE);
    // Draw red contour lines on output image.
    cv::drawContours(
      coloredRoi, contours, -1, cv::Scalar(0, 0, 255), 5, cv::LINE_8,
      hierarchy, 100);

    // For more details, refer to link below:
    // https://tinyurl.com/y5qnnxud
    float fx = camera_info.k.at(0);
    float fy = camera_info.k.at(4);

    // Getting rotated rectangle and draw the major axis
    std::vector<cv::RotatedRect> minRect(contours.size());
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
      obj_surface_depth = this->findMin(depthImg(curBoxRect)) * 0.001;

      // std::cout << "table_depth = " << table_depth << std::endl;
      // std::cout << "obj_surface_depth = " << obj_surface_depth << std::endl;

      // std::cout << "[-OBJ centroid x-] = " << x << std::endl;
      // std::cout << "[-OBJ centroid y-] = " << y << std::endl;
      // std::cout << "[-OBJ centroid z-] = " << obj_surface_depth +
      // (table_depth - obj_surface_depth) / 2 << std::endl;

      // Get estimated size of object
      // Compare the length of 2 side of the rectangle,
      // the longer side will be the major axis
      if (cv::norm(rect_points[0] - rect_points[1]) >
        cv::norm(rect_points[1] - rect_points[2]))
      {
        // Draws the major axis(red)
        cv::line(coloredRoi, pt_a, pt_b, cv::Scalar(0, 0, 255), 2);
        // Draws the minor axis (green)
        cv::line(coloredRoi, pt_c, pt_d, cv::Scalar(0, 255, 0), 2);
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

      // Mark the center point with blue dot
      cv::circle(coloredRoi, rotated_mid, 1, cv::Scalar(255, 0, 0), 1);
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
  double camera_to_plane_distance_mm,
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

    output_obj.objects[i].mask = curMask;

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
    // Generate contours.
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
    float obj_surface_depth;
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

      obj_surface_depth = this->findMin(depthImg(curBoxRect)) * 0.001;
      float x = (rotated_mid.x - ppx) / fx * obj_surface_depth;
      float y = (rotated_mid.y - ppy) / fy * obj_surface_depth;

      output_obj.objects[i].centroid.x = x;
      output_obj.objects[i].centroid.y = y;
      output_obj.objects[i].centroid.z = obj_surface_depth +
        (table_depth - obj_surface_depth) / 2;

      // Get Real Size and angle of object
      // Compare the length of 2 side of the rectangle,
      // the longer side will be the major axis
      if (cv::norm(rect_points[0] - rect_points[1]) >
        cv::norm(rect_points[1] - rect_points[2]))
      {
        // Calculates the length of the object
        output_obj.objects[i].length = obj_surface_depth * sqrt(
          pow((pt_a.x - pt_b.x) / fx, 2) +
          pow((pt_a.y - pt_b.y) / fy, 2));
        // Calculates the breadth of the object
        output_obj.objects[i].breadth = obj_surface_depth * sqrt(
          pow((pt_c.x - pt_d.x) / fx, 2) +
          pow((pt_c.y - pt_d.y) / fy, 2));
      } else {
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

      pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      segmented_cloud->header.frame_id = "camera_color_optical_frame";
      segmented_cloud->is_dense = true;

      cv::Mat tempImg = inputImg.clone();

      // Converting Depth Image to PointCloud
      for (int j = 0; j < tempFinalMask.rows; j++) {
        for (int k = 0; k < tempFinalMask.cols; k++) {
          // TODO(cardboardcode) convert segmented mask into segmented pointcloud
          int pixelValue = static_cast<int>(tempFinalMask.at<uchar>(j, k));

          if (pixelValue != 0) {
            float z = static_cast<float>(depthImg.at<uint16_t>(
                curBoxRect.y + j, curBoxRect.x + k) * 0.001);
            float x = static_cast<float>((curBoxRect.x + k - ppx) / fx) * z;
            float y = static_cast<float>((curBoxRect.y + j - ppy) / fy) * z;

            // Ignore all points that has a value of less than 0.1mm in z.
            if (std::abs(z) < 0.0001 || std::abs(z) > camera_to_plane_distance_mm * 0.001) {
              continue;
            } else {
              pcl::PointXYZ curPoint(x, y, z);
              segmented_cloud->points.push_back(curPoint);
            }
          }
        }
      }

      output_obj.objects[i].segmented_pcl = *segmented_cloud;

      // Determine object axis of segmented_pcl
      Eigen::Vector3f axis;
      Eigen::Vector4f centerpoint;
      Eigen::Vector3f eigenvalues;
      Eigen::Matrix3f eigenvectors;
      Eigen::Matrix3f covariance_matrix;

      pcl::compute3DCentroid(output_obj.objects[i].segmented_pcl, centerpoint);

      pcl::computeCovarianceMatrix(
        output_obj.objects[i].segmented_pcl,
        centerpoint,
        covariance_matrix);
      pcl::eigen33(covariance_matrix, eigenvectors, eigenvalues);

      axis = Eigen::Vector3f(
        eigenvectors.col(2)(0),
        eigenvectors.col(2)(1),
        eigenvectors.col(2)(2));

      axis = axis.normalized();

      output_obj.objects[i].axis.x = axis(0);
      output_obj.objects[i].axis.y = axis(1);
      output_obj.objects[i].axis.z = axis(2);
    }
  }
  // END of Populating EPDObjectLocalization object
  return output_obj;
}

// DEBUG
// A mutator function that will output an EPD::EPDObjectTracking object that
// contains all information required for Localization.
EPD::EPDObjectTracking P3OrtBase::infer_action(
  const cv::Mat & inputImg,
  const cv::Mat & depthImg,
  sensor_msgs::msg::CameraInfo camera_info,
  double camera_to_plane_distance_mm,
  const std::string tracker_type,
  std::vector<cv::Ptr<cv::Tracker>> & trackers,
  std::vector<int> & tracker_logs,
  std::vector<EPD::LabelledRect2d> & tracker_results,
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

  tracking_evaluate(bboxes, inputImg, tracker_type, trackers, tracker_logs, tracker_results);

  std::vector<std::string> allClassNames = this->getClassNames();
  float maskThreshold = 0.5;
  cv::Mat result = inputImg.clone();

  assert(bboxes.size() == classIndices.size());
  if (!allClassNames.empty()) {
    assert(
      allClassNames.size() >
      *std::max_element(classIndices.begin(), classIndices.end()));
  }

  // If there is zero bounding boxes generated, return empty EPDObjectTracking object.
  if (bboxes.size() == 0) {
    EPD::EPDObjectTracking output_msg(0);
    return output_msg;
  }

  EPD::EPDObjectTracking output_obj(bboxes.size());

  float table_depth = this->findMedian(depthImg) * 0.001;

  // No. of objects will be equal to number of bboxes
  /* START of Populating EPDObjectTracking object */
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

    output_obj.objects[i].mask = curMask;

    // Print out tracker number tag for object.
    for (size_t j = 0; j < tracker_results.size(); j++) {
      if (tracker_results[j].obj_bounding_box.x == curBbox[0] &&
        tracker_results[j].obj_bounding_box.y == curBbox[1] &&
        tracker_results[j].obj_bounding_box.width == curBbox[2] - curBbox[0] &&
        tracker_results[j].obj_bounding_box.height == curBbox[3] - curBbox[1])
      {
        output_obj.object_ids[i] = tracker_results[j].obj_tag;
      }
    }

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
    // Generate contours.
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
    float obj_surface_depth;
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
      }
    }
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

      obj_surface_depth = this->findMin(depthImg(curBoxRect)) * 0.001;
      float x = (rotated_mid.x - ppx) / fx * obj_surface_depth;
      float y = (rotated_mid.y - ppy) / fy * obj_surface_depth;

      output_obj.objects[i].centroid.x = x;
      output_obj.objects[i].centroid.y = y;
      output_obj.objects[i].centroid.z = obj_surface_depth +
        (table_depth - obj_surface_depth) / 2;

      // Get Real Size and angle of object
      // Compare the length of 2 side of the rectangle,
      // the longer side will be the major axis
      if (cv::norm(rect_points[0] - rect_points[1]) >
        cv::norm(rect_points[1] - rect_points[2]))
      {
        // Calculates the length of the object
        output_obj.objects[i].length = obj_surface_depth * sqrt(
          pow((pt_a.x - pt_b.x) / fx, 2) +
          pow((pt_a.y - pt_b.y) / fy, 2));
        // Calculates the breadth of the object
        output_obj.objects[i].breadth = obj_surface_depth * sqrt(
          pow((pt_c.x - pt_d.x) / fx, 2) +
          pow((pt_c.y - pt_d.y) / fy, 2));
      } else {
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

      pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      segmented_cloud->header.frame_id = "camera_color_optical_frame";
      segmented_cloud->is_dense = true;

      cv::Mat tempImg = inputImg.clone();

      // Converting Depth Image to PointCloud
      for (int j = 0; j < tempFinalMask.rows; j++) {
        for (int k = 0; k < tempFinalMask.cols; k++) {
          // TODO(cardboardcode) convert segmented mask into segmented pointcloud
          int pixelValue = static_cast<int>(tempFinalMask.at<uchar>(j, k));

          if (pixelValue != 0) {
            float z = static_cast<float>(depthImg.at<uint16_t>(
                curBoxRect.y + j, curBoxRect.x + k) * 0.001);
            float x = static_cast<float>((curBoxRect.x + k - ppx) / fx) * z;
            float y = static_cast<float>((curBoxRect.y + j - ppy) / fy) * z;

            // Ignore all points that has a value of less than 0.1mm in z.
            if (std::abs(z) < 0.0001 || std::abs(z) > camera_to_plane_distance_mm * 0.001) {
              continue;
            } else {
              pcl::PointXYZ curPoint(x, y, z);
              segmented_cloud->points.push_back(curPoint);
            }
          }
        }
      }

      output_obj.objects[i].segmented_pcl = *segmented_cloud;

      // Determine object axis of segmented_pcl
      Eigen::Vector3f axis;
      Eigen::Vector4f centerpoint;
      Eigen::Vector3f eigenvalues;
      Eigen::Matrix3f eigenvectors;
      Eigen::Matrix3f covariance_matrix;

      pcl::compute3DCentroid(output_obj.objects[i].segmented_pcl, centerpoint);

      pcl::computeCovarianceMatrix(
        output_obj.objects[i].segmented_pcl,
        centerpoint,
        covariance_matrix);
      pcl::eigen33(covariance_matrix, eigenvectors, eigenvalues);

      axis = Eigen::Vector3f(
        eigenvectors.col(2)(0),
        eigenvectors.col(2)(1),
        eigenvectors.col(2)(2));

      axis = axis.normalized();

      output_obj.objects[i].axis.x = axis(0);
      output_obj.objects[i].axis.y = axis(1);
      output_obj.objects[i].axis.z = axis(2);
    }
  }
  // END of Populating EPDObjectTracking object
  return output_obj;
}

// Filter out accurate tracked objects using both new detection results and predicted
// trackign results.
void P3OrtBase::tracking_evaluate(
  const std::vector<std::array<float, 4>> & bboxes,
  const cv::Mat & img,
  const std::string tracker_type,
  std::vector<cv::Ptr<cv::Tracker>> & trackers,
  std::vector<int> & tracker_logs,
  std::vector<EPD::LabelledRect2d> & tracker_results)
{
  if (tracker_results.size() == 0 && bboxes.size() == 0) {
    // Action 1
    // Do nothing.
    return;
  } else if (tracker_results.size() != 0 && bboxes.size() == 0) {
    // Action 2
    // If there are no detection results in frame, remove all tracking results
    // Assumption: Existing static object detections do not fluctuate and disappear for lunch.
    trackers.clear();
    tracker_results.clear();
    return;
  } else if (tracker_results.size() == 0 && bboxes.size() != 0) {
    // Action 3
    // If tracking results is empty,
    // immediately assign detection results to tracker_results with new labels.

    for (size_t i = 0; i < bboxes.size(); i++) {
      create_tracker_tag(tracker_logs);

      const auto & curBbox = bboxes[i];
      cv::Rect2d detected_box = cv::Rect2d(
        curBbox[0],
        curBbox[1],
        curBbox[2] - curBbox[0],
        curBbox[3] - curBbox[1]);

      // Create, initialize and add tracker.
      cv::Ptr<cv::Tracker> temp_tracker = create_tracker(tracker_type);
      temp_tracker->init(img, detected_box);
      trackers.push_back(temp_tracker);

      // Create LabelledRect2d object
      EPD::LabelledRect2d tracker_output;
      tracker_output.obj_tag = std::to_string(tracker_logs.size());
      tracker_output.obj_bounding_box = detected_box;
      tracker_results.push_back(tracker_output);
    }
    return;
  } else if (tracker_results.size() != 0 && bboxes.size() != 0) {
    // Action 4
    // If there is new detection results and existing tracking results,
    // determine if the new detection results are of new objects.

    // Update trackers
    for (size_t i = 0; i < trackers.size(); i++) {
      trackers[i]->update(img, tracker_results[i].obj_bounding_box);
    }

    if (tracker_results.size() > bboxes.size()) {
      // Remove tracked objects that have been removed out of frame.
      // Scan through all detection results.

      // Determine which trackers are updated.
      // If a tracker is not updated, remove it.
      std::vector<bool> updatedTrackers(tracker_results.size(), false);
      std::vector<float> trackerIOUScore(tracker_results.size(), 0.0);

      for (size_t i = 0; i < bboxes.size(); i++) {
        const auto & curBbox = bboxes[i];
        cv::Rect2d detected_box(
          curBbox[0],
          curBbox[1],
          curBbox[2] - curBbox[0],
          curBbox[3] - curBbox[1]);

        // Check through all tracking results and update tracking results.
        for (size_t j = 0; j < tracker_results.size(); j++) {
          cv::Rect2d & tracked_box = tracker_results[j].obj_bounding_box;
          // If detection results boxes has more than 0.5 intersection over union
          // (IoU), update tracker results boxes with detection results boxes.
          float iouScore = getIOU(detected_box, tracked_box);
          if (iouScore > 0.5 && iouScore > trackerIOUScore[j]) {
            tracked_box = detected_box;
            trackerIOUScore[j] = iouScore;
            updatedTrackers[j] = true;
            break;
          }
        }
      }
      // Remove all trackers that are not updated.
      for (size_t i = 0; i < tracker_results.size(); i++) {
        if (updatedTrackers[i] == false) {
          trackers.erase(trackers.begin() + i);
          tracker_results.erase(tracker_results.begin() + i);
        }
      }

    } else if (tracker_results.size() < bboxes.size()) {
      // Update existing trackers and add new object.

      // Add new detection results to existing trackers.
      // Scan through all detection results.

      // Determine which detections are new.
      // If a detection is new, add it to trackers and tracker_results.
      // If a tracker is not updated, remove it.
      std::vector<bool> newDetection(bboxes.size(), false);
      std::vector<float> trackerIOUScore(tracker_results.size(), 0.0);

      for (size_t i = 0; i < bboxes.size(); i++) {
        const auto & curBbox = bboxes[i];
        cv::Rect2d detected_box(
          curBbox[0],
          curBbox[1],
          curBbox[2] - curBbox[0],
          curBbox[3] - curBbox[1]);

        // Check through all tracking results and update tracking results.
        bool isNewDetection = true;
        for (size_t j = 0; j < tracker_results.size(); j++) {
          cv::Rect2d & tracked_box = tracker_results[j].obj_bounding_box;
          // If detection results boxes has more than 0.5 intersection over union
          // (IoU), update tracker results boxes with detection results boxes.
          float iouScore = getIOU(detected_box, tracked_box);
          if (iouScore > 0.5 && iouScore > trackerIOUScore[j]) {
            tracked_box = detected_box;
            trackerIOUScore[j] = iouScore;
            isNewDetection = false;
            break;
          }
        }
        if (isNewDetection) {
          newDetection[i] = true;
        }
      }
      // Add new detections to trackers.
      for (size_t i = 0; i < bboxes.size(); i++) {
        if (newDetection[i] == true) {
          const auto & curBbox = bboxes[i];
          cv::Rect2d detected_box(
            curBbox[0],
            curBbox[1],
            curBbox[2] - curBbox[0],
            curBbox[3] - curBbox[1]);

          create_tracker_tag(tracker_logs);

          cv::Ptr<cv::Tracker> temp_tracker = create_tracker(tracker_type);
          temp_tracker->init(img, detected_box);
          trackers.push_back(temp_tracker);

          EPD::LabelledRect2d tracker_output;
          tracker_output.obj_tag = std::to_string(tracker_logs.size());
          tracker_output.obj_bounding_box = detected_box;
          tracker_results.push_back(tracker_output);
        }
      }

    } else {
      // Update existing trackers or add and remove objects.
      std::vector<bool> updatedTrackers(tracker_results.size(), false);
      std::vector<bool> newDetection(bboxes.size(), false);
      std::vector<float> trackerIOUScore(tracker_results.size(), 0.0);

      for (size_t i = 0; i < bboxes.size(); i++) {
        const auto & curBbox = bboxes[i];
        cv::Rect2d detected_box(
          curBbox[0],
          curBbox[1],
          curBbox[2] - curBbox[0],
          curBbox[3] - curBbox[1]);

        // Check through all tracking results and update tracking results.
        bool isNewDetection = true;
        for (size_t j = 0; j < tracker_results.size(); j++) {
          cv::Rect2d & tracked_box = tracker_results[j].obj_bounding_box;
          // If detection results boxes has more than 0.5 intersection over union
          // (IoU), update tracker results boxes with detection results boxes.
          float iouScore = getIOU(detected_box, tracked_box);
          if (iouScore > 0.5 && iouScore > trackerIOUScore[j]) {
            tracked_box = detected_box;
            isNewDetection = false;
            updatedTrackers[j] = true;
            trackerIOUScore[j] = iouScore;
            break;
          }
        }
        if (isNewDetection) {
          newDetection[i] = true;
        }
      }

      // Add new detections to trackers.
      for (size_t i = 0; i < bboxes.size(); i++) {
        if (newDetection[i] == true) {
          const auto & curBbox = bboxes[i];
          cv::Rect2d detected_box(
            curBbox[0],
            curBbox[1],
            curBbox[2] - curBbox[0],
            curBbox[3] - curBbox[1]);

          create_tracker_tag(tracker_logs);

          cv::Ptr<cv::Tracker> temp_tracker = create_tracker(tracker_type);
          temp_tracker->init(img, detected_box);
          trackers.push_back(temp_tracker);

          EPD::LabelledRect2d tracker_output;
          tracker_output.obj_tag = std::to_string(tracker_logs.size());
          tracker_output.obj_bounding_box = detected_box;
          tracker_results.push_back(tracker_output);
        }
      }

      // Remove all trackers that are not updated.
      for (size_t i = 0; i < tracker_results.size(); i++) {
        if (updatedTrackers[i] == false) {
          trackers.erase(trackers.begin() + i);
          tracker_results.erase(tracker_results.begin() + i);
        }
      }
    }
    return;
  }
}

double P3OrtBase::getIOU(cv::Rect2d detected_box, cv::Rect2d tracked_box) const
{
  cv::Rect2d intersection = detected_box & tracked_box;
  return intersection.area();
}

void P3OrtBase::create_tracker_tag(std::vector<int> & tracker_logs)
{
  if (tracker_logs.size() == 0) {
    tracker_logs.push_back(0);
  } else {
    tracker_logs.push_back(tracker_logs.back() + 1);
  }
}

// Create tracker by name
cv::Ptr<cv::Tracker> P3OrtBase::create_tracker(std::string tracker_type)
{
  if (tracker_type == "KCF") {
    return cv::TrackerKCF::create();
  } else if (tracker_type == "MEDIANFLOW") {
    return cv::TrackerMedianFlow::create();
  } else if (tracker_type == "CSRT") {
    return cv::TrackerCSRT::create();
  } else {
    throw std::runtime_error(
            "Invalid OpenCV Tracker name given in usecase_config.json. "
            "Please use [KCF, MedianFlow, CSRT] only.");
  }
}

}  // namespace Ort
