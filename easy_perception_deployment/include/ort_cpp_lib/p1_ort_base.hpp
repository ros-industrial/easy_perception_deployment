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

#ifndef ORT_CPP_LIB__P1_ORT_BASE_HPP_
#define ORT_CPP_LIB__P1_ORT_BASE_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"
#include "ort_cpp_lib/ort_base.hpp"

namespace Ort
{
/*! \class P1OrtBase
    \brief An Precision-Level 1 (P1) ONNXRuntime (Ort) Base class object.
    This class object instantiates a Precision Level 1 Ort Session which takes a
    typical ONNX model used for solely image classification and
    runs an inferenc engine.
*/
class P1OrtBase : public OrtBase
{
public:
  /*! \brief A fixed minimal image size needed for a lower bound requirement
  for image classification of adequate result.*/
  static constexpr int64_t MIN_IMAGE_SIZE = 800;
  /*! \brief A Constructor function*/
  P1OrtBase(
    float ratio,
    int newW,
    int newH,
    int paddedW,
    int paddedH,
    const uint16_t numClasses,
    const std::string & modelPath,
    const boost::optional<size_t> & gpuIdx = boost::none,
    const boost::optional<std::vector<std::vector<int64_t>>> &
    inputShapes = boost::none);
  /*! \brief A Destructor function*/
  ~P1OrtBase();
  /*! \brief A Mutator function that runs the P1 Ort Session and gets P1
  inference result.*/
  std::vector<std::string> infer(const cv::Mat & inputImg);
  /*! \brief A Getter function that gets the number of object names used for an
  ongoing session.*/
  uint16_t getNumClasses() const {return m_numClasses;}
  /*! \brief A Getter function that gets the list of object text labels which
  will be used for outputting visualized inference result or for specific use-case
  filters.*/
  const std::vector<std::string> & getClassNames() const {return m_classNames;}
  /*! \brief A Mutator function that sets the list of object text labels to be
  used for the P1 Ort Session.*/
  void initClassNames(const std::vector<std::string> & classNames);

private:
  /*! \brief The number of object text labels given an input label list.*/
  const uint16_t m_numClasses;
  /*! \brief The aspect ratio calculated from the dimension of an input image
  frame, which is provided when the first input image is received by EasyPerceptionDeployment.*/
  float m_ratio;
  /*! \brief The new padded frame dimensions of an input image. This is used for
  P2 and P3 object detection.*/
  int m_newW, m_newH, m_paddedW, m_paddedH;
  /*! \brief A vector of object text labels given an input label list.*/
  std::vector<std::string> m_classNames;

  /*! \brief A Mutator function that converts a 3-layered 2D RGB input image
  into a 1D input data tensor to be passed to the Ort Session for processing.\n
  This variant takes a generic input image represented by a char pointer.
  */
  void preprocess(
    float * dst,
    const unsigned char * src,
    const int64_t targetImgWidth,
    const int64_t targetImgHeight,
    const size_t numChannels,
    const std::vector<float> & meanVal = {},
    const std::vector<float> & stdVal = {}) const;
  /*! \brief An Mutator function that takes the inference output and determines
  the most possible object identity given an input label list.
  */
  std::vector<std::pair<int, float>> getTopKRaw(
    const std::vector<float *> & inferenceOutput,
    const uint16_t k = 1,
    const bool useSoftmax = true) const;

  /*! \brief A Mutator function that takes the inference output and determines
  the most possible object identity given an input label list.
  */
  std::vector<std::string> processTopK(
    const std::vector<float *> & inferenceOutput,
    const uint16_t k = 1,
    const bool useSoftmax = true) const;
};
}  // namespace Ort

#endif  // ORT_CPP_LIB__P1_ORT_BASE_HPP_
