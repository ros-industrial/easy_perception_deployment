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

#ifndef ORT_CPP_LIB__P2_ORT_BASE_HPP_
#define ORT_CPP_LIB__P2_ORT_BASE_HPP_

#include <optional>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "ort_cpp_lib/ort_base.hpp"
#include "epd_utils_lib/message_utils.hpp"


namespace Ort
{
/*! \class P2OrtBase
    \brief An Precision-Level 2 (P2) ONNXRuntime (Ort) Base class object.
    This class object instantiates a Precision Level 2 Ort Session which takes a
    typical ONNX model used for solely object detection and
    runs an inference engine.
*/
class P2OrtBase : public OrtBase
{
public:
  /*! \brief A fixed minimal image size needed for a lower bound requirement
  for image classification of adequate result.*/
  static constexpr int64_t MIN_IMAGE_SIZE = 800;
  /*! \brief A Constructor function*/
  P2OrtBase(
    float ratio,
    int newW,
    int newH,
    int paddedW,
    int paddedH,
    const uint16_t numClasses, const std::string & modelPath,
    const boost::optional<size_t> & gpuIdx = boost::none,
    const boost::optional<std::vector<std::vector<int64_t>>> &
    inputShapes = boost::none);
  /*! \brief A Destructor function*/
  ~P2OrtBase();
  /*! \brief A auxillary Mutator function that calls the internal overloading
  infer_action function.*/
  EPD::EPDObjectDetection infer_action(const cv::Mat & inputImg);

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
  This variant takes a generic input image represented by a conventional opencv
  Matrix.
  */
  void preprocess(
    float * dst,
    const cv::Mat & imgSrc,
    const int64_t targetImgWidth,
    const int64_t targetImgHeight,
    const int numChannels) const;

  /*! \brief A Mutator function that runs a P2 Ort Session and gets P2
  inference result for use by external agents.*/
  EPD::EPDObjectDetection infer_action(
    const cv::Mat & inputImg,
    int newW,
    int newH,
    int paddedW,
    int paddedH,
    float ratio,
    float * dst,
    float confThresh,
    const cv::Scalar & meanVal);
};
}  // namespace Ort

#endif  // ORT_CPP_LIB__P2_ORT_BASE_HPP_
