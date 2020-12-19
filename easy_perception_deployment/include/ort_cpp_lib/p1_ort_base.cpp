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
#include <numeric>
#include <string>
#include <vector>
#include <utility>

#include "p1_ort_base.hpp"

void softmax(float * input, const size_t inputLen)
{
  const float maxVal = *std::max_element(input, input + inputLen);

  const float sum = std::accumulate(
    input, input + inputLen, 0.0,
    [&](float a, const float b) {return std::move(a) + expf(b - maxVal);});

  const float offset = maxVal + logf(sum);
  for (auto it = input; it != (input + inputLen); ++it) {
    *it = expf(*it - offset);
  }
}

namespace Ort
{
// Constructor
P1OrtBase::P1OrtBase(
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
P1OrtBase::~P1OrtBase() {}

// Mutator 4
std::vector<std::string> P1OrtBase::infer(const cv::Mat & inputImg)
{
  cv::Mat tmpImg;
  cv::resize(inputImg, tmpImg, cv::Size(m_newW, m_newH));

  static constexpr int64_t IMG_CHANNEL = 3;
  float * dst = new float[m_newW * m_newH * IMG_CHANNEL];

  static const std::vector<float> IMAGENET_MEAN = {0.406, 0.456, 0.485};
  static const std::vector<float> IMAGENET_STD = {0.225, 0.224, 0.229};

  this->preprocess(
    dst, tmpImg.data, m_newW, m_newH, IMG_CHANNEL,
    IMAGENET_MEAN, IMAGENET_STD);
  auto inferenceOutput = (*this)({reinterpret_cast<float *>(dst)});

  const int TOP_K = 1;

  return this->processTopK({inferenceOutput[0].first}, TOP_K);
}

// Mutator 3
void P1OrtBase::initClassNames(const std::vector<std::string> & classNames)
{
  if (classNames.size() != m_numClasses) {
    throw std::runtime_error("Mismatch number of classes\n");
  }
  m_classNames = classNames;
}

// Mutator 1
void P1OrtBase::preprocess(
  float * dst,
  const unsigned char * src,
  const int64_t targetImgWidth,
  const int64_t targetImgHeight,
  const size_t numChannels,
  const std::vector<float> & meanVal,
  const std::vector<float> & stdVal) const
{
  /* Check if meanVal and stdVal vector arrays are empty.
  If true, check if meanVal size corresponds to stdVal size.
  If true, also check if meanVal size corresponds to the num of channels which is 3.
  Otherwise, nothing. */
  if (!meanVal.empty() && !stdVal.empty()) {
    assert(meanVal.size() == stdVal.size() && meanVal.size() == numChannels);
  }

  int64_t dataLength = targetImgHeight * targetImgWidth * numChannels;

  // Note that src in this case is input image.
  memcpy(dst, reinterpret_cast<const float *>(src), dataLength);


  if (!meanVal.empty() && !stdVal.empty()) {
    for (int i = 0; i < targetImgHeight; ++i) {
      for (int j = 0; j < targetImgWidth; ++j) {
        for (size_t c = 0; c < numChannels; ++c) {
          dst[c * targetImgHeight * targetImgWidth + i * targetImgWidth + j] =
            (src[i * targetImgWidth * numChannels + j * numChannels + c] /
            255.0 - meanVal[c]) / stdVal[c];
        }
      }
    }
  } else {
    for (int i = 0; i < targetImgHeight; ++i) {
      for (int j = 0; j < targetImgWidth; ++j) {
        for (size_t c = 0; c < numChannels; ++c) {
          dst[c * targetImgHeight * targetImgWidth + i * targetImgWidth + j] =
            src[i * targetImgWidth * numChannels + j * numChannels + c] /
            255.0;
        }
      }
    }
  }
}

std::vector<std::pair<int, float>>
P1OrtBase::getTopKRaw(
  const std::vector<float *> & inferenceOutput,
  const uint16_t k,
  const bool useSoftmax) const
{
  const uint16_t realK = std::max(std::min(k, m_numClasses), static_cast<uint16_t>(1));

  assert(inferenceOutput.size() == 1);
  float * processData = inferenceOutput[0];
  if (useSoftmax) {
    softmax(processData, m_numClasses);
  }

  std::vector<std::pair<int, float>> ps;
  ps.reserve(m_numClasses);

  for (int i = 0; i < m_numClasses; ++i) {
    ps.emplace_back(std::make_pair(i, processData[i]));
  }

  std::sort(
    ps.begin(), ps.end(), [](const auto & elem1, const auto & elem2)
    {return elem1.second > elem2.second;});

  return std::vector<std::pair<int, float>>(ps.begin(), ps.begin() + realK);
}

std::vector<std::string>
P1OrtBase::processTopK(
  const std::vector<float *> & inferenceOutput,
  const uint16_t k,
  const bool useSoftmax) const
{
  auto ps = this->getTopKRaw(inferenceOutput, k, useSoftmax);

  std::vector<std::string> topK_obj_identities;
  for (const auto & elem : ps) {
    topK_obj_identities.push_back(m_classNames[elem.first]);
  }

  return topK_obj_identities;
}
}  // namespace Ort
