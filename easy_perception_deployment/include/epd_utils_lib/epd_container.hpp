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

#ifndef EPD_UTILS_LIB__EPD_CONTAINER_HPP_
#define EPD_UTILS_LIB__EPD_CONTAINER_HPP_

#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <string>
#include <vector>

#include "ort_cpp_lib/ort_base.hpp"
#include "ort_cpp_lib/p2_ort_base.hpp"
#include "ort_cpp_lib/p3_ort_base.hpp"
#include "epd_utils_lib/message_utils.hpp"


namespace EPD
{
/*! \class EPDContainer
    \brief An Easy Perception Deployment(EPD) Container class object.
    The EPDContainer class object parses the session_config.json and
    usecase_config.json files to determine how an ONNX model can be
    deployed as an inference engine using the ONNXRuntime library.
*/
class EPDContainer
{
public:
  /*! \brief An pointer for a Precision Level 3 OrtBase object*/
  Ort::P3OrtBase * p3_ort_session;
  /*! \brief An pointer for a Precision Level 2 OrtBase object*/
  Ort::P2OrtBase * p2_ort_session;
  /*! \brief The determined precision_level for an input ONNX model file,
  * stated by the session_config.json. */
  unsigned int precision_level;
  /*! \brief A fixed integer for expected RGB 2D images*/
  const int IMG_CHANNEL = 3;
  /*! \brief The constant filepath to session_config.json*/
  const std::string PATH_TO_SESSION_CONFIG = PATH_TO_PACKAGE "/config/session_config.json";
  /*! \brief The constant filepath to usecase_config.json*/
  const std::string PATH_TO_USECASE_CONFIG = PATH_TO_PACKAGE "/config/usecase_config.json";
  /*! \brief The filepath to a template color image for Color-Matching use-case
  * filter.
  */
  std::string template_color_path;
  /*! \brief The filepath to the essential class label list, that maps classIndices
  * to human-understandable object text labels.
  */
  std::string class_label_path;
  /*! \brief The filepath to an input ONNX model file*/
  std::string onnx_model_path;

  /*! \brief The selected use-case mode. Values can only be 0,1,2.\n
  *  See usecase_config.hpp for more details.\n
  */
  unsigned int useCaseMode;

  /*! \brief A subset of selected object names from an input label list, used
  * for Counting use-case filter.
  */
  std::vector<std::string> countClassNames;
  /*! \brief A string name for user-selected OpenCV tracker, used
  * for Tracking use-case filter.
  */
  std::vector<cv::Ptr<cv::Tracker>> trackers;
  std::string tracker_type;
  std::vector<int> tracker_logs;
  std::vector<EPD::LabelledRect2d> tracker_results;

  bool requestAddressed;
  /*! \brief A list of human-understandable object text labels from input
  * label list.
  */
  std::vector<std::string> classNames;

  /*! \brief A Constructor function*/
  EPDContainer(void);
  /*! \brief A Destructor function*/
  ~EPDContainer(void);

  /*! \brief A Getter function that gets the bool variable, hasInitialized*/
  bool isInit(void);
  /*! \brief A Getter function that gets the bool variable, onlyVisualize*/
  bool isVisualize(void);
  /*! \brief A Getter function that gets the bool variable, isService*/
  bool isService(void);
  /*! \brief A Getter function that gets the int variable, frame_height*/
  int getHeight(void);
  /*! \brief A Getter function that gets the int variable, frame_width*/
  int getWidth(void);
  /*! \brief A Mutator function that sets the bool variable, hasInitialized*/
  void setInitBoolean(bool input);
  /*! \brief A Mutator function that sets the int variables, frame_width & frame_height*/
  void setFrameDimension(int width, int height);
  /*! \brief A Mutator function that sets the appropriate precision-Level
  *   specific OrtBase object.
  */
  void initORTSessionHandler();

  cv::Mat visualize(
    const EPD::EPDObjectDetection result,
    const cv::Mat input_image);

  cv::Mat visualize(
    const EPD::EPDObjectTracking result,
    const cv::Mat input_image);

private:
  /*! \brief A boolean to indicate that OrtBase object has been initialized.*/
  bool hasInitialized;
  /*! \brief A boolean to determine the type of final user output.*/
  bool onlyVisualize;
  /*! \brief A boolean to determine whether EPD acts
  as a service or a publisher*/
  bool onlyService;
  /*! \brief Expected dimensions of the data provided by an input camera.*/
  int frame_width, frame_height;

  /*! \brief A Mutator function that parses the session_config.json file.*/
  void setModelConfigFile();
  /*! \brief A Mutator function that parses the usecase_config.json file.*/
  void setUseCaseConfigFile();
  /*! \brief A Mutator function that does a test initialization of an
  *  OrtBase object based on an input ONNX model file and determines the
  *  expected precision level.
  */
  void setPrecisionLevel();
  /*! \brief A Mutator function that parses the input label list .txt file into
  *  the variable, classNames.
  */
  void setLabelList();
};

}  // namespace EPD

#endif  // EPD_UTILS_LIB__EPD_CONTAINER_HPP_
