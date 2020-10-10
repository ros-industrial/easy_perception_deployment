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

#ifndef EPD_UTILS_LIB__MESSAGE_UTILS_HPP_
#define EPD_UTILS_LIB__MESSAGE_UTILS_HPP_

#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

namespace EPD
{
/*! \class EPDObjectDetection
    \brief An Easy Perception Deployment (EPD) ObjectDetection class object.
    This object functions as a transient container of inference results to
    transport them for processing in by Processor class object.
*/
class EPDObjectDetection
{
public:
  /*! \brief A vector of bounding boxes with xmin, ymin, xmax, ymax.*/
  std::vector<std::array<float, 4>> bboxes;
  /*! \brief A vector of indices that indicate the numerical identities of
  corresponding bounding boxes of the same index.
  */
  std::vector<uint64_t> classIndices;
  /*! \brief A vector of indices that indicate the float confidence scores of
  corresponding bounding boxes of the same index.
  */
  std::vector<float> scores;
  /*! \brief A vector of image-encoded 32FC1 greyscale masks for P3 results only.
  */
  std::vector<cv::Mat> masks;

  /*! \brief A set size for all vectors in class.*/
  size_t data_size;

  /*! \brief A Constructor function. This object can only be called a known
  size to minimize memory use for storage.*/
  explicit EPDObjectDetection(size_t input_size)
  {
    data_size = input_size;

    bboxes.reserve(input_size);
    classIndices.reserve(input_size);
    scores.reserve(input_size);
    masks.reserve(input_size);
  }

private:
};
}  // namespace EPD

#endif  // EPD_UTILS_LIB__MESSAGE_UTILS_HPP_
