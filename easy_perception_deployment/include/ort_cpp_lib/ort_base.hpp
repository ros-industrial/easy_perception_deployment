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
#ifndef ORT_CPP_LIB__ORT_BASE_HPP_
#define ORT_CPP_LIB__ORT_BASE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "boost/none.hpp"
#include "boost/optional.hpp"


namespace Ort
{
/*! \class OrtBase
    \brief An ONNXRuntime (Ort) Base class object.
    This is the base class for P1OrtBase, P2OrtBase and P3OrtBase. It serves an
    auxillary class object to directly interface with the ONNXRuntime CPP API
    to instantiate an Ort session to run as an inference engine.
*/
class OrtBase
{
public:
  /*! \brief A Constructor function*/
  OrtBase(
    const std::string & modelPath,  //
    const boost::optional<size_t> & gpuIdx = boost::none,
    const boost::optional<std::vector<std::vector<std::int64_t>>> &
    inputShapes = boost::none);
  /*! \brief A Destructor function*/
  ~OrtBase();
  /*! \brief A convienence datatype to store output inference result.*/
  using DataOutputType = std::pair<float *, std::vector<std::int64_t>>;
  /*! \brief A Mutator operator function that conducts inference with
  preprocessed input image data.
  */
  std::vector<DataOutputType> operator()(const std::vector<float *> & inputImgData);
  /*! \brief A Getter function that gets the number of outputs which is
  used to determine the level of precision in EPDContainer class object.*/
  int getNumOutputs(void);

private:
  /*! \brief An internal class object that interfaces with Ort CPP API.*/
  class OrtBaseImpl;
  /*! \brief A pointer to the OrtBaseImpl*/
  std::unique_ptr<OrtBaseImpl> base_impl_;
};

}  // namespace Ort

#endif  // ORT_CPP_LIB__ORT_BASE_HPP_
