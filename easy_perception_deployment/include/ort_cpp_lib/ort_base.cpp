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
#include <functional>
#include <utility>
#include <cassert>
#include <numeric>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

#include "ort_base.hpp"
#include "onnxruntime/core/session/onnxruntime_cxx_api.h"

#if USE_GPU
#include "onnxruntime/core/providers/cuda/cuda_provider_factory.h"
#endif

template<typename T, template<typename, typename = std::allocator<T>> class Container>
std::ostream & operator<<(std::ostream & os, const Container<T> & container)
{
  using ContainerType = Container<T>;
  for (typename ContainerType::const_iterator it = container.begin();
    it != container.end();
    ++it)
  {
    os << *it << " ";
  }
  return os;
}

namespace Ort
{

class OrtBase::OrtBaseImpl
{
public:
  OrtBaseImpl(
    const std::string & modelPath,         //
    const boost::optional<size_t> & gpuIdx,  //
    const boost::optional<std::vector<std::vector<int64_t>>> & inputShapes);
  ~OrtBaseImpl();

  int getNumOutputs(void);
  std::vector<DataOutputType> operator()(const std::vector<float *> & inputData);

private:
  void initSession();
  void initModelInfo();

  Ort::Session m_session;
  Ort::Env m_env;
  Ort::AllocatorWithDefaultOptions m_ortAllocator;

  boost::optional<size_t> m_gpuIdx;

  std::vector<char *> m_inputNodeNames;
  std::vector<char *> m_outputNodeNames;
  std::vector<std::vector<int64_t>> m_inputShapes;
  std::vector<std::vector<int64_t>> m_outputShapes;

  std::vector<int64_t> m_inputTensorSizes;
  std::vector<int64_t> m_outputTensorSizes;

  uint8_t m_numInputs;
  uint8_t m_numOutputs;
  std::string m_modelPath;
  bool m_inputShapesProvided = false;
};

// Constructor
OrtBase::OrtBase(
  const std::string & modelPath,
  const boost::optional<size_t> & gpuIdx,
  const boost::optional<std::vector<std::vector<int64_t>>> & inputShapes)
: base_impl_(std::make_unique<OrtBaseImpl>(modelPath, gpuIdx, inputShapes))
{}

// Destructor
OrtBase::~OrtBase() = default;

std::vector<OrtBase::DataOutputType> OrtBase::operator()(const std::vector<float *> & inputImgData)
{
  return this->base_impl_->operator()(inputImgData);
}

int OrtBase::getNumOutputs()
{
  return base_impl_->getNumOutputs();
}

// Constructor
OrtBase::OrtBaseImpl::OrtBaseImpl(
  const std::string & modelPath,         //
  const boost::optional<size_t> & gpuIdx,  //
  const boost::optional<std::vector<std::vector<int64_t>>> & inputShapes)
: m_session(nullptr),
  m_env(nullptr),
  m_ortAllocator(),
  m_gpuIdx(gpuIdx),
  m_inputNodeNames(),
  m_outputNodeNames(),
  m_inputShapes(),
  m_outputShapes(),
  m_numInputs(0),
  m_numOutputs(0),
  m_modelPath(modelPath)
{
  this->initSession();

  if (inputShapes.is_initialized() && !inputShapes->empty()) {
    m_inputShapesProvided = true;
    m_inputShapes = inputShapes.value();
  }

  this->initModelInfo();
}

OrtBase::OrtBaseImpl::~OrtBaseImpl()
{
  for (auto & elem : this->m_inputNodeNames) {
    free(elem);
    elem = nullptr;
  }
  this->m_inputNodeNames.clear();

  for (auto & elem : this->m_outputNodeNames) {
    free(elem);
    elem = nullptr;
  }
  this->m_outputNodeNames.clear();
}

int OrtBase::OrtBaseImpl::getNumOutputs()
{
  return unsigned(m_numOutputs);
}

void OrtBase::OrtBaseImpl::initSession()
{
  m_env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "Ort");
  Ort::SessionOptions sessionOptions;

  /* TODO(cardboardcode) Need to take care of the following line
  as it is related to CPU
  consumption using openmp */
  // sessionOptions.SetIntraOpNumThreads(1);

  #if USE_GPU
  if (m_gpuIdx.is_initialized()) {
    Ort::ThrowOnError(
      OrtSessionOptionsAppendExecutionProvider_CUDA(
        sessionOptions,
        m_gpuIdx.value()));
  }
  #endif

  sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  m_session = Ort::Session(m_env, m_modelPath.c_str(), sessionOptions);
  m_numInputs = m_session.GetInputCount();

  m_inputNodeNames.reserve(m_numInputs);
  m_inputTensorSizes.reserve(m_numInputs);

  m_numOutputs = m_session.GetOutputCount();

  m_outputNodeNames.reserve(m_numOutputs);
  m_outputTensorSizes.reserve(m_numOutputs);
}

void OrtBase::OrtBaseImpl::initModelInfo()
{
  for (int i = 0; i < m_numInputs; i++) {
    // If m_inputShapes not initialized,
    // then look at m_session and derive.
    // Ensures that m_inputShapes is filled properly before use.
    if (!m_inputShapesProvided) {
      Ort::TypeInfo typeInfo = m_session.GetInputTypeInfo(i);
      auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();

      m_inputShapes.emplace_back(tensorInfo.GetShape());
    }

    const auto & curInputShape = m_inputShapes[i];

    m_inputTensorSizes.emplace_back(
      std::accumulate(
        std::begin(curInputShape),
        std::end(curInputShape),
        1,
        std::multiplies<int64_t>()));

    // DEBUG
    // Identified potential failure point for loading point.
    char * inputName = m_session.GetInputName(i, m_ortAllocator);
    m_inputNodeNames.emplace_back(strdup(inputName));
    m_ortAllocator.Free(inputName);
  }

  {
  #if PRINT_MODEL_INFO
    std::stringstream ssInputs;
    ssInputs << "Input shapes: ";
    ssInputs << m_inputShapes << std::endl;
    ssInputs << "Input node names: ";
    ssInputs << m_inputNodeNames << std::endl;
    ssInputs << "Input length: ";
    ssInputs << unsigned(m_numInputs) << std::endl;
    printf("%s\n", ssInputs.str().c_str());
  #endif
  }

  for (int i = 0; i < m_numOutputs; ++i) {
    Ort::TypeInfo typeInfo = m_session.GetOutputTypeInfo(i);
    auto tensorInfo = typeInfo.GetTensorTypeAndShapeInfo();

    m_outputShapes.emplace_back(tensorInfo.GetShape());

    char * outputName = m_session.GetOutputName(i, m_ortAllocator);
    m_outputNodeNames.emplace_back(strdup(outputName));
    m_ortAllocator.Free(outputName);
  }

  {
  #if PRINT_MODEL_INFO
    std::stringstream ssOutputs;
    ssOutputs << "Output shapes: ";
    ssOutputs << m_outputShapes << std::endl;
    ssOutputs << "Output node names: ";
    ssOutputs << m_outputNodeNames << std::endl;
    ssOutputs << "Output length: ";
    ssOutputs << unsigned(m_numOutputs) << std::endl;
    printf("%s\n", ssOutputs.str().c_str());
  #endif
  }
  // TODO(cardboardcode) To allow flexible printing of model info for DEBUG.
  // DEBUG Print out model output shapes and node names.
  // std::stringstream ssOutputs;
  // ssOutputs << "Model output shapes: ";
  // ssOutputs << m_outputShapes << std::endl;
  // ssOutputs << "Model output node names: ";
  // ssOutputs << m_outputNodeNames << std::endl;
  // ssOutputs << "Model output length: ";
  // ssOutputs << unsigned(m_numOutputs) << std::endl;
  // std::cout << ssOutputs.str().c_str()  << std::endl;
}

// Run ORT session on processed input image.
std::vector<OrtBase::DataOutputType> OrtBase::OrtBaseImpl::operator()(
  const std::vector<float *> & inputData)
{
  if (m_numInputs != inputData.size()) {
    throw std::runtime_error("Mismatch size of input data\n");
  }
  // Investigate if this statement means it is using CPU instead of GPU when GPU is intended.
  Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  // Create inputTensors
  std::vector<Ort::Value> inputTensors;
  inputTensors.reserve(m_numInputs);
  // Populate inputTensors with device-specific memoryInfo, the input image and the inputShapes.
  for (int i = 0; i < m_numInputs; ++i) {
    inputTensors.emplace_back(
      std::move(
        Ort::Value::CreateTensor<float>(
          memoryInfo,
          const_cast<float *>(inputData[i]),
          m_inputTensorSizes[i],
          m_inputShapes[i].data(),
          m_inputShapes[i].size())));
  }
  // INFERENCE DONE HERE.
  auto outputTensors = m_session.Run(
    Ort::RunOptions{nullptr},
    m_inputNodeNames.data(),
    inputTensors.data(),
    m_numInputs,
    m_outputNodeNames.data(),
    m_numOutputs);

  // Check if outputTensors is empty. It should not be, even if it is garbage.
  assert(outputTensors.size() == m_numOutputs);

  std::vector<DataOutputType> outputData;
  outputData.reserve(m_numOutputs);

  for (auto & elem : outputTensors) {
    outputData.emplace_back(
      std::make_pair(
        std::move(elem.GetTensorMutableData<float>()),
        elem.GetTensorTypeAndShapeInfo().GetShape()));
  }
  return outputData;
}

}  // namespace Ort
