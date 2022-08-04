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

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "p2_ort_base.hpp"
#include "epd_utils_lib/usecase_config.hpp"
#include "epd_utils_lib/message_utils.hpp"

namespace Ort
{

// Constructor
P2OrtBase::P2OrtBase(
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
P2OrtBase::~P2OrtBase() {}

EPD::EPDObjectDetection P2OrtBase::infer_action(const cv::Mat & inputImg)
{
  std::vector<float> dst(3 * m_paddedH * m_paddedW);

  return this->infer_action(
    inputImg, m_newW, m_newH,
    m_paddedW, m_paddedH, m_ratio,
    dst.data(), 0.5, cv::Scalar(102.9801, 115.9465, 122.7717));
}

void P2OrtBase::initClassNames(const std::vector<std::string> & classNames)
{
  if (classNames.size() != m_numClasses) {
    throw std::runtime_error("Mismatch number of classes\n");
  }
  m_classNames = classNames;
}

void P2OrtBase::preprocess(
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
EPD::EPDObjectDetection P2OrtBase::infer_action(
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

  // boxes, labels, scores
  auto inferenceOutput = (*this)({dst});

  assert(inferenceOutput[1].second.size() == 1);
  size_t nBoxes = inferenceOutput[1].second[0];

  std::vector<std::array<int, 4>> bboxes;
  std::vector<uint64_t> classIndices;
  std::vector<float> scores;

  bboxes.reserve(nBoxes);
  classIndices.reserve(nBoxes);
  scores.reserve(nBoxes);

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
    }
  }

  if (bboxes.size() == 0) {
    // Provide warning of empty
    EPD::EPDObjectDetection output_msg(0);
    return output_msg;
  }

  EPD::EPDObjectDetection output_obj(bboxes.size());
  output_obj.bboxes = bboxes;
  output_obj.classIndices = classIndices;
  output_obj.scores = scores;

  return output_obj;
}

}  // namespace Ort
