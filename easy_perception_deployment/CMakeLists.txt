cmake_minimum_required(VERSION 3.5)
project(easy_perception_deployment)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(epd_onnxruntime_vendor REQUIRED)

# Json CPP libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

# Include external ROS packages.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(epd_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(include
  ${PCL_INCLUDE_DIRS}
)

# Add all custom library headers for compilation.
set(EPD_UTILS
  include/epd_utils_lib/epd_container.cpp

  include/ort_cpp_lib/ort_base.cpp
  include/ort_cpp_lib/p3_ort_base.cpp
  include/ort_cpp_lib/p2_ort_base.cpp
)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/data/model/MaskRCNN-10.onnx")
  message(AUTHOR_WARNING "Pretrained models are downloaded.")
else()
  message(AUTHOR_WARNING "Downloading pretrained ONNX model from ONNX Model Zoo.")
  file(DOWNLOAD https://github.com/onnx/models/raw/main/validated/vision/classification/squeezenet/model/squeezenet1.1-7.onnx
    ${CMAKE_CURRENT_LIST_DIR}/data/model/squeezenet1.1-7.onnx)
  file(DOWNLOAD https://github.com/onnx/models/raw/main/validated/vision/object_detection_segmentation/faster-rcnn/model/FasterRCNN-10.onnx
    ${CMAKE_CURRENT_LIST_DIR}/data/model/FasterRCNN-10.onnx)
  file(DOWNLOAD
  https://github.com/onnx/models/raw/main/validated/vision/object_detection_segmentation/mask-rcnn/model/MaskRCNN-10.onnx
    ${CMAKE_CURRENT_LIST_DIR}/data/model/MaskRCNN-10.onnx)
endif()

# Check if CUDA is available in local onnxruntime build
find_package(CUDA)

if(CUDA_FOUND)
  message(AUTHOR_WARNING "Using [-GPU-].")
  add_definitions(-DUSE_GPU=true)
else()
  message(AUTHOR_WARNING "Your local onnxruntime does not support CUDA. Using [-CPU-] instead.")
  add_definitions(-DUSE_GPU=false)
endif()

if(DEBUG)
  message(AUTHOR_WARNING "Running in DEBUG mode.")
  add_definitions(-DPRINT_MODEL_INFO=true)
else()
  add_definitions(-DPRINT_MODEL_INFO=false)
endif()

add_definitions(-DPATH_TO_PACKAGE="${CMAKE_CURRENT_LIST_DIR}")
add_definitions(${PCL_DEFINITIONS})

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  # Excluding to avoid conflicting Doxygen python code blocks.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_pep257
    ament_cmake_flake8
    ament_cmake_lint_cmake
  )

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(epd_test_init test/test_init.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_init epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_init ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_invalid_usecase test/test_invalid_usecase.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_invalid_usecase epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_invalid_usecase ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_P2_counting_action test/p2u1a.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P2_counting_action epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_P2_counting_action ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_P2_counting_visualize test/p2u1v.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P2_counting_visualize epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_P2_counting_visualize ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_P2_colormatch_action test/p2u2a.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P2_colormatch_action epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_P2_colormatch_action ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_P3_counting_visualize test/p3u1v.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_counting_visualize epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_counting_visualize ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_P3_colormatch_visualize test/p3u2v.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_colormatch_visualize epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_colormatch_visualize ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_P3_colormatch_action test/p3u2a.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_colormatch_action epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_colormatch_action ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_P3_localize_action test/p3u3a.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_localize_action epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_localize_action ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_P3_localize_visualize test/p3u3v.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_localize_visualize epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_localize_visualize ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_P3_tracking_visualize test/p3u4v.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_tracking_visualize epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_tracking_visualize ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_P3_tracking_action test/p3u4a.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_tracking_action epd_onnxruntime_vendor OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_tracking_action ${JSONCPP_LIBRARIES})

  ament_add_gtest(epd_test_easy_perception_deployment test/test_easy_perception_deployment.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_easy_perception_deployment epd_onnxruntime_vendor rclcpp std_msgs sensor_msgs epd_msgs OpenCV cv_bridge message_filters pcl_conversions)
  target_link_libraries(epd_test_easy_perception_deployment ${JSONCPP_LIBRARIES})

endif()

add_executable(easy_perception_deployment src/main.cpp ${EPD_UTILS})
ament_target_dependencies(
  easy_perception_deployment
  epd_onnxruntime_vendor
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  epd_msgs
  OpenCV
  cv_bridge
  message_filters
  pcl_conversions
)
target_link_libraries(easy_perception_deployment ${PCL_LIBRARIES} ${JSONCPP_LIBRARIES})


install(TARGETS
  easy_perception_deployment
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
