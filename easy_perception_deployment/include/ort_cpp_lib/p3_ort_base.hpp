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

#ifndef ORT_CPP_LIB__P3_ORT_BASE_HPP_
#define ORT_CPP_LIB__P3_ORT_BASE_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <optional>
#include <string>
#include <vector>

#include "ort_cpp_lib/ort_base.hpp"
#include "epd_utils_lib/message_utils.hpp"
#include "sensor_msgs/msg/camera_info.hpp"


namespace Ort
{
/*! \class P3OrtBase
    \brief An Precision-Level 3 (P3) ONNXRuntime (Ort) Base class object.
    This class object instantiates a Precision Level 3 Ort Session which takes a
    typical ONNX model used for solely object detection and
    runs an inference engine.
*/
class P3OrtBase : public OrtBase
{
public:
  /*! \brief A fixed minimal image size needed for a lower bound requirement
  for image classification of adequate result.*/
  static constexpr int64_t MIN_IMAGE_SIZE = 800;
  /*! \brief A Constructor function*/
  P3OrtBase(
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
  ~P3OrtBase();
  /*! \brief A auxillary Mutator function that calls the internal overloading
  infer_visualize function.*/
  cv::Mat infer_visualize(const cv::Mat & inputImg);

  /*! \brief A auxillary Mutator function that calls the internal overloading
  infer_visualize function with depth_msg and camera_info.*/
  cv::Mat infer_visualize(
    const cv::Mat & inputImg,
    const cv::Mat & depthImg,
    sensor_msgs::msg::CameraInfo camera_info);
  /*! \brief A auxillary Mutator function that calls the internal overloading
  infer_visualize function with depth_msg and camera_info.*/
  cv::Mat infer_visualize(
    const cv::Mat & inputImg,
    const cv::Mat & depthImg,
    sensor_msgs::msg::CameraInfo camera_info,
    const std::string tracker_type,
    std::vector<cv::Ptr<cv::Tracker>> & trackers,
    std::vector<int> & tracker_logs,
    std::vector<EPD::LabelledRect2d> & tracker_results);

  /*! \brief A auxillary Mutator function that calls the internal overloading
  infer_action function.*/
  EPD::EPDObjectDetection infer_action(const cv::Mat & inputImg);

  /*! \brief A auxillary Mutator function that calls the internal overloading
  infer_action function.*/
  EPD::EPDObjectLocalization infer_action(
    const cv::Mat & inputImg,
    const cv::Mat & depthImg,
    sensor_msgs::msg::CameraInfo camera_info,
    double camera_to_plane_distance_mm);

  /*! \brief A auxillary Mutator function that calls the internal overloading
  infer_action function.*/
  EPD::EPDObjectTracking infer_action(
    const cv::Mat & inputImg,
    const cv::Mat & depthImg,
    sensor_msgs::msg::CameraInfo camera_info,
    double camera_to_plane_distance_mm,
    const std::string tracker_type,
    std::vector<cv::Ptr<cv::Tracker>> & trackers,
    std::vector<int> & tracker_logs,
    std::vector<EPD::LabelledRect2d> & tracker_results);

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

  /*! \brief A Mutator function that runs a P3 Ort Session and gets P3
  inference result for visualization purposes.*/
  cv::Mat infer_visualize(
    const cv::Mat & inputImg,
    int newW,
    int newH,
    int paddedW,
    int paddedH,
    float ratio,
    float * dst,
    float confThresh,
    const cv::Scalar & meanVal);

  /*! \brief A Mutator function that runs a P3 Ort Session and gets P3
  inference result for visualization purposes with localization results.*/
  cv::Mat infer_visualize(
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
    const cv::Scalar & meanVal);

  /*! \brief A Mutator function that runs a P3 Ort Session and gets P3
  inference result for visualization purposes with localization results.*/
  cv::Mat infer_visualize(
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
    const cv::Scalar & meanVal);

  /*! \brief A Mutator function that runs a P3 Ort Session and gets P3
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

  /*! \brief A Mutator function that runs a P3 Ort Session and gets P3
  inference result with Localization results for use by external agents.*/
  EPD::EPDObjectLocalization infer_action(
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
    const cv::Scalar & meanVal);

  /*! \brief A Mutator function that runs a P3 Ort Session and gets P3
  inference result with Tracking results for use by external agents.*/
  EPD::EPDObjectTracking infer_action(
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
    const cv::Scalar & meanVal);

  /*! \brief A Mutator function that takes P2 inference outputs and illustrates
  derived bounding boxes with corresponding object labels for visualization
  purposes.*/
  cv::Mat visualize(
    const cv::Mat & img,
    const std::vector<std::array<float, 4>> & bboxes,
    const std::vector<uint64_t> & classIndices,
    const std::vector<cv::Mat> & masks,
    const std::vector<std::string> & allClassNames,
    const float maskThreshold);

  /*! \brief A Getter function that returns the median z-value of the scene.*/
  double findMedian(cv::Mat depthImg);
  /*! \brief A Getter function that returns the largest z-value of the scene.*/
  double findMin(cv::Mat depthImg);
  /*! \brief A Getter function that returns the Intersection-over-Union (IoU)
  area between two Rect2d objects.*/
  double getIOU(cv::Rect2d detected_box, cv::Rect2d tracked_box) const;
  /*! \brief A Mutator function that add a user-defined OpenCV tracker. The
  tracker can be KCF, MedianFlow or CSRT.*/
  cv::Ptr<cv::Tracker> create_tracker(std::string tracker_type);
  /*! \brief A Mutator function that create an integer tracker tag and add it
  to a session-persistent log of tracker tags.*/
  void create_tracker_tag(std::vector<int> & tracker_logs);

  /*! \brief A Mutator function that updates, adds or removes tracked
  objects by comparing trackers to new detections.*/
  void tracking_evaluate(
    const std::vector<std::array<float, 4>> & bboxes,
    const cv::Mat & img,
    const std::string tracker_type,
    std::vector<cv::Ptr<cv::Tracker>> & trackers,
    std::vector<int> & tracker_logs,
    std::vector<EPD::LabelledRect2d> & tracker_results);

  /*! \brief A Mutator function that takes P3 inference outputs and illustrates
  derived bounding boxes with corresponding object labels for visualization
  purposes and localization.*/
  cv::Mat localize_visualize(
    const cv::Mat & img,
    const cv::Mat & depthImg,
    sensor_msgs::msg::CameraInfo camera_info,
    const std::vector<std::array<float, 4>> & bboxes,
    const std::vector<uint64_t> & classIndices,
    const std::vector<cv::Mat> & masks,
    const std::vector<std::string> & allClassNames,
    const float maskThreshold);

  /*! \brief A Mutator function that takes P3 inference outputs and illustrates
  derived bounding boxes with corresponding object labels for visualization
  purposes and tracked localization.*/
  cv::Mat tracking_visualize(
    const cv::Mat & img,
    const cv::Mat & depthImg,
    sensor_msgs::msg::CameraInfo camera_info,
    std::vector<EPD::LabelledRect2d> & tracker_results,
    const std::vector<std::array<float, 4>> & bboxes,
    const std::vector<uint64_t> & classIndices,
    const std::vector<cv::Mat> & masks,
    const std::vector<std::string> & allClassNames,
    const float maskThreshold);
};
}  // namespace Ort

#endif  // ORT_CPP_LIB__P3_ORT_BASE_HPP_
