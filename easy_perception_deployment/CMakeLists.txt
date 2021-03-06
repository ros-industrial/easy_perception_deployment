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

# Include onnxruntime C++ API Library
set(onnxruntime_INSTALL_PREFIX /usr/local)
set(onnxruntime_INCLUDE_DIRS
  ${onnxruntime_INSTALL_PREFIX}/include/onnxruntime
  ${onnxruntime_INSTALL_PREFIX}/include/onnxruntime/core/session
)
find_library(onnxruntime_LIBS NAMES onnxruntime PATHS /usr/local/lib)

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
  ${onnxruntime_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add all custom library headers for compilation.
set(EPD_UTILS
  include/epd_utils_lib/epd_container.cpp

  include/ort_cpp_lib/ort_base.cpp
  include/ort_cpp_lib/p3_ort_base.cpp
  include/ort_cpp_lib/p2_ort_base.cpp
  include/ort_cpp_lib/p1_ort_base.cpp
)

# Check if CUDA is available in local onnxruntime build
include(CheckLanguage)
check_language(CUDA)

if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/data/model/squeezenet1.1-7.onnx")
  message(AUTHOR_WARNING "Pretrained models are downloaded.")
else()
  message(AUTHOR_WARNING "Downloading pretrained ONNX model from ONNX Model Zoo.")
  execute_process(COMMAND wget
  https://github.com/onnx/models/raw/master/vision/classification/squeezenet/model/squeezenet1.1-7.onnx
  --directory-prefix=${CMAKE_CURRENT_LIST_DIR}/data/model/)
  execute_process(COMMAND wget
  https://github.com/onnx/models/raw/master/vision/object_detection_segmentation/faster-rcnn/model/FasterRCNN-10.onnx
  --directory-prefix=${CMAKE_CURRENT_LIST_DIR}/data/model/)
  execute_process(COMMAND wget
  https://github.com/onnx/models/raw/master/vision/object_detection_segmentation/mask-rcnn/model/MaskRCNN-10.onnx
  --directory-prefix=${CMAKE_CURRENT_LIST_DIR}/data/model/)
endif()

if(EXISTS ${CMAKE_CUDA_COMPILER})
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
  )

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(epd_test_init test/test_init.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_init OpenCV cv_bridge)
  target_link_libraries(epd_test_init ${onnxruntime_LIBS})

  ament_add_gtest(epd_test_invalid_usecase test/test_invalid_usecase.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_invalid_usecase OpenCV cv_bridge)
  target_link_libraries(epd_test_invalid_usecase ${onnxruntime_LIBS})

  ament_add_gtest(epd_test_P1_classification_action test/p1u0a.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P1_classification_action OpenCV cv_bridge)
  target_link_libraries(epd_test_P1_classification_action ${onnxruntime_LIBS})

  ament_add_gtest(epd_test_P2_counting_action test/p2u1a.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P2_counting_action OpenCV cv_bridge)
  target_link_libraries(epd_test_P2_counting_action ${onnxruntime_LIBS})

  ament_add_gtest(epd_test_P2_counting_visualize test/p2u1v.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P2_counting_visualize OpenCV cv_bridge)
  target_link_libraries(epd_test_P2_counting_visualize ${onnxruntime_LIBS})

  ament_add_gtest(epd_test_P2_colormatch_action test/p2u2a.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P2_colormatch_action OpenCV cv_bridge)
  target_link_libraries(epd_test_P2_colormatch_action ${onnxruntime_LIBS})

  ament_add_gtest(epd_test_P3_counting_visualize test/p3u1v.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_counting_visualize OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_counting_visualize ${onnxruntime_LIBS})

  ament_add_gtest(epd_test_P3_colormatch_visualize test/p3u2v.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_colormatch_visualize OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_colormatch_visualize ${onnxruntime_LIBS})

  ament_add_gtest(epd_test_P3_colormatch_action test/p3u2a.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_colormatch_action OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_colormatch_action ${onnxruntime_LIBS})

  ament_add_gtest(epd_test_P3_localize_action test/p3u3a.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_localize_action OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_localize_action ${onnxruntime_LIBS})

  ament_add_gtest(epd_test_P3_localize_visualize test/p3u3v.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_P3_localize_visualize OpenCV cv_bridge)
  target_link_libraries(epd_test_P3_localize_visualize ${onnxruntime_LIBS})

  ament_add_gtest(epd_test_processor test/test_processor.cpp ${EPD_UTILS})
  ament_target_dependencies(epd_test_processor rclcpp std_msgs sensor_msgs epd_msgs OpenCV cv_bridge message_filters pcl_conversions)
  target_link_libraries(epd_test_processor ${onnxruntime_LIBS})

endif()

add_executable(processor src/processor.cpp ${EPD_UTILS})
ament_target_dependencies(
  processor
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
target_link_libraries(processor ${onnxruntime_LIBS} ${PCL_LIBRARIES})


install(TARGETS
  processor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
