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

// ROS2 LIB
#include <memory>
#include "rclcpp/rclcpp.hpp"

// EPD_UTILS LIB
#include "epd_utils_lib/easy_perception_deployment.hpp"

int main(int argc, char * argv[])
{
  setlinebuf(stdout);
  rclcpp::init(argc, argv);

  auto epd_node = std::make_shared<EasyPerceptionDeployment>();

  rclcpp::spin(epd_node);
  rclcpp::shutdown();
  return 0;
}
