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


#ifndef EPD_UTILS_LIB__EASY_PERCEPTION_DEPLOYMENT_HPP_
#define EPD_UTILS_LIB__EASY_PERCEPTION_DEPLOYMENT_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <functional>

// OpenCV LIB
#include "opencv2/opencv.hpp"

// ROS2 LIB
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

// EPD_UTILS LIB
#include "epd_utils_lib/epd_container.hpp"
#include "epd_msgs/msg/epd_image_classification.hpp"
#include "epd_msgs/msg/epd_object_detection.hpp"
#include "epd_msgs/msg/epd_object_localization.hpp"
#include "epd_msgs/msg/epd_object_tracking.hpp"
#include "epd_msgs/msg/localized_object.hpp"
#include "epd_msgs/srv/perception.hpp"
#include "epd_utils_lib/usecase_config.hpp"
#include "epd_utils_lib/message_utils.hpp"

#include "pcl_conversions/pcl_conversions.h"
/*! \class EasyPerceptionDeployment
    \brief An EasyPerceptionDeployment class object.
    This class object inherits rclcpp::Node object and acts the main bridge
    between the ROS2 interface and the underlying ort_cpp_lib library that is
    based on ONNXRuntime Library.
*/
class EasyPerceptionDeployment : public rclcpp::Node
{
public:
  /*! \brief A Constructor function*/
  EasyPerceptionDeployment(void);
  /*! \brief A function that abstracts processing of input image in image_callback.*/
  void process_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const;
  /*! \brief A function that abstracts processing of input image in localize_callback.*/
  void process_localize_callback(
    const sensor_msgs::msg::Image::SharedPtr msg,
    const sensor_msgs::msg::Image::SharedPtr depth_msg,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);
  /*! \brief A function that abstracts processing of input image in tracking_callback.*/
  void process_tracking_callback(
    const sensor_msgs::msg::Image::SharedPtr msg,
    const sensor_msgs::msg::Image::SharedPtr depth_msg,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

private:
  /*! \brief A subscriber member variable to receive 2D RGB images to receive.*/
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

  /*! \brief An alias definition for SyncPolicy that is used below for sync_ object.*/
  typedef message_filters::sync_policies::ApproximateTime
    <sensor_msgs::msg::Image, sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo> SyncPolicy;

  /*! \brief A policy-synchronized subscriber member variable
  to receive rectified 2D RGB images.
  */
  message_filters::Subscriber<sensor_msgs::msg::Image> localize_image_rgb;
  /*! \brief A policy-synchronized subscriber member variable
  to receive rectified 2D Depth images.
  */
  message_filters::Subscriber<sensor_msgs::msg::Image> localize_image_depth;
  /*! \brief A policy-synchronized subscriber member variable
  to receive camera information.
  */
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> localize_cam_info;
  /*! \brief A Synchronizer policy member variable.*/
  message_filters::Synchronizer<SyncPolicy> sync_;

  /*! \brief A publisher member variable to output visualization of inference
  results*/
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visual_pub;
  /*! \brief A publisher member variable to output Precision-Level 1 (P1)
  specific inference output suitable for external agents.*/
  rclcpp::Publisher<epd_msgs::msg::EPDImageClassification>::SharedPtr p1_pub;
  /*! \brief A publisher member variable to output Precision-Level 2 (P2)
  specific inference output suitable for external agents.*/
  rclcpp::Publisher<epd_msgs::msg::EPDObjectDetection>::SharedPtr p2_pub;
  /*! \brief A publisher member variable to output Precision-Level 3 (P3)
  specific inference output suitable for external agents.*/
  rclcpp::Publisher<epd_msgs::msg::EPDObjectDetection>::SharedPtr p3_pub;
  /*! \brief A publisher member variable to output Precision-Level 3 (P3)
  specific inference output suitable for external agents.*/
  rclcpp::Publisher<epd_msgs::msg::EPDObjectLocalization>::SharedPtr localize_pub;

  rclcpp::Publisher<epd_msgs::msg::EPDObjectTracking>::SharedPtr tracking_pub;

  rclcpp::Service<epd_msgs::srv::Perception>::SharedPtr srv_;
  /*! \brief A singular EPDContainer object that deploys a user-defined
  ONNX model as an inference enginer using onnxruntime.
  */
  mutable EPD::EPDContainer ortAgent_;

  /*! \brief A ROS2 callback function utilized by Localization Use case.\n
  It gets ortAgent_ to initialize once and only once when the first input image
  is received.\n
  It also populates the appropriate ROS messages with EPDObjectLocalization
  when the onlyVisualize boolean flag is set to false.\n
  */
  void localize_callback(
    const sensor_msgs::msg::Image::SharedPtr msg,
    const sensor_msgs::msg::Image::SharedPtr depth_msg,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

  /*! \brief A ROS2 callback function utilized by Tracking Use case.\n
  It gets ortAgent_ to initialize once and only once when the first input image
  is received.\n
  It also populates the appropriate ROS messages with EPDObjectTracking
  when the onlyVisualize boolean flag is set to false.\n
  */
  void tracking_callback(
    const sensor_msgs::msg::Image::SharedPtr msg,
    const sensor_msgs::msg::Image::SharedPtr depth_msg,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

  /*! \brief A ROS2 callback function utilized by Classification Use case.\n
  It gets ortAgent_ to initialize once and only once when the first input image
  is received.\n
  It also populates the appropriate ROS messages with EPDObjectDetection
  when the onlyVisualize boolean flag is set to false.\n
  */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const;

  /*! \brief A function that throws an error if there is a unexpected change in
  camera frame dimensions. This is done to safeguard an OrtSession and allow it
  to fail clearly.
  */
  void hasCameraChanged(
    const int img_height,
    const int img_width) const;

  /*! \brief A mutator function that checks if an OrtSession is initialized or not.
  If it is, check if camera frame dimensions is unchanged. Refer to
  hasCameraChanged function.
  Otherwise, initialize an OrtSession.
  */
  void checkOrtAgentIsInitialized(
    const int img_height,
    const int img_width) const;
};

EasyPerceptionDeployment::EasyPerceptionDeployment(void)
: Node("easy_perception_deployment"),
  localize_image_rgb(this, "/camera/color/image_raw"),
  localize_image_depth(this, "/camera/depth/image_rect_raw"),
  localize_cam_info(this, "/camera/color/camera_info"),
  sync_(SyncPolicy(10), localize_image_rgb, localize_image_depth, localize_cam_info)
{
  // Creating Subscriber to get Input Image.
  image_sub = this->create_subscription<sensor_msgs::msg::Image>(
    "/easy_perception_deployment/image_input",
    rclcpp::SensorDataQoS(),
    std::bind(&EasyPerceptionDeployment::image_callback, this, std::placeholders::_1));

  // Creating Publisher to output Visualizable P2 and P3 Detection Results.
  visual_pub = this->create_publisher<sensor_msgs::msg::Image>(
    "/easy_perception_deployment/image_output",
    10);
  // Creating Publisher to output Action P1 Detection Results.
  p1_pub = this->create_publisher<epd_msgs::msg::EPDImageClassification>(
    "/easy_perception_deployment/epd_p1_output",
    10);
  // Creating Publisher to output Action P2 Detection Results.
  p2_pub = this->create_publisher<epd_msgs::msg::EPDObjectDetection>(
    "/easy_perception_deployment/epd_p2_output",
    10);
  // Creating Publisher to output Action P3 Detection Results.
  p3_pub = this->create_publisher<epd_msgs::msg::EPDObjectDetection>(
    "/easy_perception_deployment/epd_p3_output",
    10);
  // Creating Publisher to output Action P3 and Localization Detection Results.
  localize_pub = this->create_publisher<epd_msgs::msg::EPDObjectLocalization>(
    "/easy_perception_deployment/epd_localize_output",
    10);
  // Creating Publisher to output Action P3 and Tracking Detection Results.
  tracking_pub = this->create_publisher<epd_msgs::msg::EPDObjectTracking>(
    "/easy_perception_deployment/epd_tracking_output",
    10);

  // If useCaseMode is detected to be Localization or Tracking,
  // Subscribe to all synchronized ROS2 topics.
  if (ortAgent_.useCaseMode == 3) {
    localize_image_rgb.subscribe();
    localize_image_depth.subscribe();
    localize_cam_info.subscribe();
    sync_.registerCallback(&EasyPerceptionDeployment::localize_callback, this);
    image_sub.reset();
  } else if (ortAgent_.useCaseMode == 4) {
    localize_image_rgb.subscribe();
    localize_image_depth.subscribe();
    localize_cam_info.subscribe();
    sync_.registerCallback(&EasyPerceptionDeployment::tracking_callback, this);
    image_sub.reset();
  } else {
    localize_image_rgb.unsubscribe();
    localize_image_depth.unsubscribe();
    localize_cam_info.unsubscribe();
  }

  this->declare_parameter("camera_to_plane_distance_mm");
  this->set_parameter(rclcpp::Parameter("camera_to_plane_distance_mm", 1000.0));

  auto handle_emd_request =
    [this](
    const std::shared_ptr<epd_msgs::srv::Perception::Request> request,
    std::shared_ptr<epd_msgs::srv::Perception::Response> response) -> void
    {
      (void)request;
      RCLCPP_INFO(this->get_logger(), "[ RECEIVED ] - EMD Grasp-Planner Request");
      response->success = true;

      if (ortAgent_.useCaseMode == 4) {
        response->tracking_enabled = true;
      } else {
        response->tracking_enabled = false;
      }

      if (ortAgent_.useCaseMode == 3) {
        localize_image_rgb.subscribe();
        localize_image_depth.subscribe();
        localize_cam_info.subscribe();
        sync_.registerCallback(&EasyPerceptionDeployment::localize_callback, this);
      } else if (ortAgent_.useCaseMode == 4) {
        localize_image_rgb.subscribe();
        localize_image_depth.subscribe();
        localize_cam_info.subscribe();
        sync_.registerCallback(&EasyPerceptionDeployment::tracking_callback, this);
      } else {
        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
          "/easy_perception_deployment/image_input",
          10,
          std::bind(&EasyPerceptionDeployment::image_callback, this, std::placeholders::_1));
      }

      ortAgent_.requestAddressed = false;
    };

  srv_ = this->create_service<epd_msgs::srv::Perception>(
    "epd_perception_service",
    handle_emd_request);

  // Log all session_config and usecase_config configurations for user to check on system boot
  RCLCPP_INFO(this->get_logger(), "[-ONNX Model-] - %s", ortAgent_.onnx_model_path.c_str());
  RCLCPP_INFO(this->get_logger(), "[-Label List-] - %s", ortAgent_.class_label_path.c_str());
  RCLCPP_INFO(this->get_logger(), "[-Precision Level-] - %d", ortAgent_.precision_level);
  if (ortAgent_.isVisualize()) {
    RCLCPP_INFO(this->get_logger(), "[-Mode-] - VISUALISE");
  } else {
    RCLCPP_INFO(this->get_logger(), "[-Mode-] - ACTION");
  }

  switch (ortAgent_.useCaseMode) {
    case EPD::CLASSIFICATION_MODE:
      RCLCPP_INFO(this->get_logger(), "[-Use Case-] - EPD::CLASSIFICATION_MODE");
      break;
    case EPD::COUNTING_MODE:
      RCLCPP_INFO(this->get_logger(), "[-Use Case-] - EPD::COUNTING_MODE");
      break;
    case EPD::COLOR_MATCHING_MODE:
      RCLCPP_INFO(this->get_logger(), "[-Use Case-] - EPD::COLOR_MATCHING_MODE");
      break;
    case EPD::LOCALISATION_MODE:
      RCLCPP_INFO(this->get_logger(), "[-Use Case-] - EPD::LOCALISATION_MODE");
      RCLCPP_INFO(this->get_logger(), "[- Input RGB Image Topic -] - /camera/color/image_raw");
      RCLCPP_INFO(
        this->get_logger(),
        "[- Input Depth Image Topic -] - "
        "/camera/aligned_depth_to_color/image_raw");
      RCLCPP_INFO(this->get_logger(), "[- Camera Info Topic -] - /camera/color/camera_info");
      break;
    case EPD::TRACKING_MODE:
      RCLCPP_INFO(this->get_logger(), "[-Use Case-] - EPD::TRACKING_MODE");
      break;
  }
}

void EasyPerceptionDeployment::hasCameraChanged(const int img_height, const int img_width) const
{
  // TODO(cardboardcode) Implement auto reinitialization of Ort Session.
  /*
  Check if height and width has changed or not.
  If either dim changed, throw runtime error.
  Otherwise, proceed.
  */
  if (ortAgent_.getWidth() != img_width && ortAgent_.getHeight() != img_height) {
    throw std::runtime_error("Input camera changed. Please restart.");
  }
}

void EasyPerceptionDeployment::checkOrtAgentIsInitialized(
  const int img_height,
  const int img_width) const
{
  if (!ortAgent_.isInit()) {
    ortAgent_.setFrameDimension(img_width, img_height);
    ortAgent_.initORTSessionHandler();
    ortAgent_.setInitBoolean(true);
  } else {
    hasCameraChanged(img_height, img_width);
  }
}

void EasyPerceptionDeployment::process_localize_callback(
  const sensor_msgs::msg::Image::SharedPtr msg,
  const sensor_msgs::msg::Image::SharedPtr depth_msg,
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  if (ortAgent_.requestAddressed) {
    return;
  }

  rclcpp::Parameter double_param = this->get_parameter("camera_to_plane_distance_mm");
  double camera_to_plane_distance_mm = double_param.as_double();
  /* Check if input image is empty or not.
  If empty, discard image and don't process.
  Otherwise, proceed with processing.
  */
  if (msg->height == 0) {
    RCLCPP_WARN(this->get_logger(), "Input image empty. Discarding.");
    return;
  }

  // Convert ROS Image message to cv::Mat for processing.
  std::shared_ptr<cv_bridge::CvImage> imgptr = cv_bridge::toCvCopy(msg, "bgr8");
  cv::Mat img = imgptr->image;

  std::shared_ptr<cv_bridge::CvImage> depth_imageptr = cv_bridge::toCvCopy(
    depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  cv::Mat depth_img = depth_imageptr->image;

  checkOrtAgentIsInitialized(img.rows, img.cols);

  // Initialize timer
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

  EPD::EPDObjectLocalization result = ortAgent_.p3_ort_session->infer(
    img,
    depth_img,
    *camera_info,
    camera_to_plane_distance_mm);

  cv::Mat resultImg;
  if (ortAgent_.isVisualize()) {
    EPD::EPDObjectTracking converted_result(result.data_size);
    converted_result.object_ids.clear();
    for (size_t i = 0; i < result.data_size; i++) {
      converted_result.objects.emplace_back(result.objects[i]);
    }

    cv::Mat resultImg = ortAgent_.visualize(converted_result, img);

    sensor_msgs::msg::Image::SharedPtr output_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resultImg).toImageMsg();
    visual_pub->publish(*output_msg);

    // DEBUG
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
    RCLCPP_INFO(this->get_logger(), "[-FPS-]= %f\n", 1000.0 / elapsedTime.count());

  } else {
    epd_msgs::msg::EPDObjectLocalization output_msg;

    output_msg.header = std_msgs::msg::Header();
    output_msg.header.frame_id = "camera_color_optical_frame";
    output_msg.frame_width = img.cols;
    output_msg.frame_height = img.rows;
    output_msg.depth_image = *depth_msg;

    // Populate ppx, ppy, fx and fy intrinsic camera parameters
    output_msg.ppx = camera_info->k.at(2);
    output_msg.fx = camera_info->k.at(0);
    output_msg.ppy = camera_info->k.at(5);
    output_msg.fy = camera_info->k.at(4);

    // Populate output_msg objects and roi_array
    for (size_t i = 0; i < result.data_size; i++) {
      epd_msgs::msg::LocalizedObject object;
      object.name = result.objects[i].name;
      object.roi = result.objects[i].roi;
      sensor_msgs::msg::Image::SharedPtr mask_ptr = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        "mono16",
        result.objects[i].mask).toImageMsg();
      object.segmented_binary_mask = *mask_ptr;
      object.segmented_binary_mask.header.frame_id = "camera_color_optical_frame";
      object.centroid = result.objects[i].centroid;
      object.length = result.objects[i].length;
      object.breadth = result.objects[i].breadth;
      object.height = result.objects[i].height;
      object.axis = result.objects[i].axis;

      sensor_msgs::msg::PointCloud2 output_segmented_pcl;
      pcl::toROSMsg(result.objects[i].segmented_pcl, output_segmented_pcl);
      object.segmented_pcl = output_segmented_pcl;

      output_msg.objects.push_back(object);
    }
    // DEBUG
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
    RCLCPP_INFO(this->get_logger(), "[-FPS-]= %f\n", 1000.0 / elapsedTime.count());

    output_msg.process_time = elapsedTime.count();

    localize_pub->publish(output_msg);
  }

  if (ortAgent_.isService()) {
    localize_image_rgb.subscribe();
    localize_image_depth.subscribe();
    localize_cam_info.subscribe();
    ortAgent_.requestAddressed = true;
  }
}

// WARNING: The use of message filter sychronization causes the intake of image
// from a realsense D415 or D435 camera to be irregular. In other words, this callback cannot
// be called at a fixed interval.
void EasyPerceptionDeployment::localize_callback(
  const sensor_msgs::msg::Image::SharedPtr msg,
  const sensor_msgs::msg::Image::SharedPtr depth_msg,
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  this->process_localize_callback(msg, depth_msg, camera_info);
}

void EasyPerceptionDeployment::process_tracking_callback(
  const sensor_msgs::msg::Image::SharedPtr msg,
  const sensor_msgs::msg::Image::SharedPtr depth_msg,
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  if (ortAgent_.requestAddressed) {
    return;
  }

  rclcpp::Parameter double_param = this->get_parameter("camera_to_plane_distance_mm");
  double camera_to_plane_distance_mm = double_param.as_double();
  /* Check if input image is empty or not.
  If empty, discard image and don't process.
  Otherwise, proceed with processing.
  */
  if (msg->height == 0) {
    RCLCPP_WARN(this->get_logger(), "Input image empty. Discarding.");
    return;
  }

  // Convert ROS Image message to cv::Mat for processing.
  std::shared_ptr<cv_bridge::CvImage> imgptr = cv_bridge::toCvCopy(msg, "bgr8");
  cv::Mat img = imgptr->image;

  std::shared_ptr<cv_bridge::CvImage> depth_imageptr = cv_bridge::toCvCopy(
    depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  cv::Mat depth_img = depth_imageptr->image;

  checkOrtAgentIsInitialized(img.rows, img.cols);

  // Initialize timer
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

  cv::Mat resultImg;

  EPD::EPDObjectTracking result = ortAgent_.p3_ort_session->infer(
    img,
    depth_img,
    *camera_info,
    camera_to_plane_distance_mm,
    ortAgent_.tracker_type,
    ortAgent_.trackers,
    ortAgent_.tracker_logs,
    ortAgent_.tracker_results);

  if (ortAgent_.isVisualize()) {
    resultImg = ortAgent_.visualize(result, img);

    sensor_msgs::msg::Image::SharedPtr output_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resultImg).toImageMsg();
    visual_pub->publish(*output_msg);

    // DEBUG
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
    RCLCPP_INFO(this->get_logger(), "[-FPS-]= %f\n", 1000.0 / elapsedTime.count());

  } else {
    epd_msgs::msg::EPDObjectTracking output_msg;

    output_msg.header = std_msgs::msg::Header();
    output_msg.header.frame_id = "camera_color_optical_frame";
    output_msg.frame_width = img.cols;
    output_msg.frame_height = img.rows;
    output_msg.depth_image = *depth_msg;

    // Populate ppx, ppy, fx and fy intrinsic camera parameters
    output_msg.ppx = camera_info->k.at(2);
    output_msg.fx = camera_info->k.at(0);
    output_msg.ppy = camera_info->k.at(5);
    output_msg.fy = camera_info->k.at(4);

    // Populate output_msg objects and roi_array
    for (size_t i = 0; i < result.data_size; i++) {
      epd_msgs::msg::LocalizedObject object;
      object.name = result.objects[i].name;
      object.roi = result.objects[i].roi;
      sensor_msgs::msg::Image::SharedPtr mask_ptr = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        "mono16",
        result.objects[i].mask).toImageMsg();
      object.segmented_binary_mask = *mask_ptr;
      object.segmented_binary_mask.header.frame_id = "camera_color_optical_frame";
      object.centroid = result.objects[i].centroid;
      object.length = result.objects[i].length;
      object.breadth = result.objects[i].breadth;
      object.height = result.objects[i].height;
      object.axis = result.objects[i].axis;

      sensor_msgs::msg::PointCloud2 output_segmented_pcl;
      pcl::toROSMsg(result.objects[i].segmented_pcl, output_segmented_pcl);
      object.segmented_pcl = output_segmented_pcl;

      output_msg.object_ids.push_back(result.object_ids[i]);
      output_msg.objects.push_back(object);
    }
    // DEBUG
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
    RCLCPP_INFO(this->get_logger(), "[-FPS-]= %f\n", 1000.0 / elapsedTime.count());

    output_msg.process_time = elapsedTime.count();

    tracking_pub->publish(output_msg);
  }

  if (ortAgent_.isService()) {
    localize_image_rgb.subscribe();
    localize_image_depth.subscribe();
    localize_cam_info.subscribe();
    ortAgent_.requestAddressed = true;
  }
}

void EasyPerceptionDeployment::tracking_callback(
  const sensor_msgs::msg::Image::SharedPtr msg,
  const sensor_msgs::msg::Image::SharedPtr depth_msg,
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  this->process_tracking_callback(msg, depth_msg, camera_info);
}

void EasyPerceptionDeployment::process_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
const
{
  /* Check if input image is empty or not.
  If empty, discard image and don't process.
  Otherwise, proceed with processing.
  */
  if (msg->height == 0) {
    RCLCPP_WARN(this->get_logger(), "Input image empty. Discarding.");
    return;
  }

  // Convert ROS Image message to cv::Mat for processing.
  std::shared_ptr<cv_bridge::CvImage> imgptr = cv_bridge::toCvCopy(msg, "bgr8");
  cv::Mat img = imgptr->image;

  checkOrtAgentIsInitialized(img.rows, img.cols);
  // DEBUG

  // Initialize timer
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

  cv::Mat resultImg;
  switch (ortAgent_.precision_level) {
    case 2:
      {
        EPD::EPDObjectDetection result = ortAgent_.p2_ort_session->infer(img);
        EPD::activateUseCase(
          img,
          result.bboxes,
          result.classIndices,
          result.scores,
          result.masks,
          ortAgent_.classNames,
          ortAgent_.useCaseMode,
          ortAgent_.countClassNames,
          ortAgent_.template_color_path);

        EPD::EPDObjectDetection output_obj(result.bboxes.size());
        output_obj.bboxes = result.bboxes;
        output_obj.classIndices = result.classIndices;
        output_obj.scores = result.scores;

        if (ortAgent_.isVisualize()) {
          resultImg = ortAgent_.visualize(output_obj, img);
          sensor_msgs::msg::Image::SharedPtr output_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resultImg).toImageMsg();
          visual_pub->publish(*output_msg);
        } else {
          epd_msgs::msg::EPDObjectDetection output_msg;
          for (size_t i = 0; i < output_obj.data_size; i++) {
            output_msg.class_indices.push_back(output_obj.classIndices[i]);

            output_msg.scores.push_back(output_obj.scores[i]);

            sensor_msgs::msg::RegionOfInterest roi;
            roi.x_offset = output_obj.bboxes[i][0];
            roi.y_offset = output_obj.bboxes[i][1];
            roi.width = output_obj.bboxes[i][2] - output_obj.bboxes[i][0];
            roi.height = output_obj.bboxes[i][3] - output_obj.bboxes[i][1];
            roi.do_rectify = false;
            output_msg.bboxes.push_back(roi);
          }
          p2_pub->publish(output_msg);
        }

        break;
      }
    case 3:
      {
        EPD::EPDObjectDetection result = ortAgent_.p3_ort_session->infer(img);
        EPD::activateUseCase(
          img,
          result.bboxes,
          result.classIndices,
          result.scores,
          result.masks,
          ortAgent_.classNames,
          ortAgent_.useCaseMode,
          ortAgent_.countClassNames,
          ortAgent_.template_color_path);

        EPD::EPDObjectDetection output_obj(result.bboxes.size());
        output_obj.bboxes = result.bboxes;
        output_obj.classIndices = result.classIndices;
        output_obj.scores = result.scores;
        output_obj.masks = result.masks;

        if (ortAgent_.isVisualize()) {
          resultImg = ortAgent_.visualize(output_obj, img);
          sensor_msgs::msg::Image::SharedPtr output_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resultImg).toImageMsg();
          visual_pub->publish(*output_msg);
        } else {
          epd_msgs::msg::EPDObjectDetection output_msg;
          for (size_t i = 0; i < output_obj.data_size; i++) {
            output_msg.class_indices.push_back(output_obj.classIndices[i]);

            output_msg.scores.push_back(output_obj.scores[i]);

            sensor_msgs::msg::RegionOfInterest roi;
            roi.x_offset = output_obj.bboxes[i][0];
            roi.y_offset = output_obj.bboxes[i][1];
            roi.width = output_obj.bboxes[i][2] - output_obj.bboxes[i][0];
            roi.height = output_obj.bboxes[i][3] - output_obj.bboxes[i][1];
            roi.do_rectify = false;
            output_msg.bboxes.push_back(roi);

            sensor_msgs::msg::Image::SharedPtr mask =
              cv_bridge::CvImage(
              std_msgs::msg::Header(),
              "32FC1",
              output_obj.masks[i]).toImageMsg();
            output_msg.masks.push_back(*mask);
          }
          p3_pub->publish(output_msg);
        }

        break;
      }
  }

  // DEBUG
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
  RCLCPP_INFO(this->get_logger(), "[-FPS-]= %f\n", 1000.0 / elapsedTime.count());
}

void EasyPerceptionDeployment::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
{
  this->process_image_callback(msg);
}

#endif  // EPD_UTILS_LIB__EASY_PERCEPTION_DEPLOYMENT_HPP_
