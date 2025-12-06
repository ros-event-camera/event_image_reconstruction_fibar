#
# Copyright 2025 Bernd Pfrommer <bernd.pfrommer@eventvisionresearch.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(event_image_reconstruction_fibar)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(fibar_lib REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
# add_definitions(-DRESCALE)
if(${image_transport_VERSION} VERSION_GREATER_EQUAL "6.3.0")
  add_definitions(-DIMAGE_TRANSPORT_USE_QOS)
endif()
if(${image_transport_VERSION} VERSION_GREATER_EQUAL "6.4.0")
  add_definitions(-DIMAGE_TRANSPORT_USE_NODEINTERFACE)
endif()
if(${image_transport_VERSION} VERSION_GREATER_EQUAL "4.0.0")
  add_definitions(-DIMAGE_TRANSPORT_USE_PUB_OPTIONS)
endif()
if(${rclcpp_VERSION} VERSION_GREATER_EQUAL "28.0.0")
  add_definitions(-DUSE_MATCHED_EVENTS)
endif()

add_library(fibar
  src/fibar.cpp)

target_link_libraries(fibar
  cv_bridge::cv_bridge
  ${event_camera_codecs_TARGETS}
  ${event_camera_msgs_TARGETS}
  fibar_lib::fibar_lib
  image_transport::image_transport
  opencv_core
  opencv_imgcodecs
  rclcpp::rclcpp
  rclcpp_components::component
  rosbag2_cpp::rosbag2_cpp
  ${sensor_msgs_TARGETS})

# target_compile_options(fibar PRIVATE -fsanitize=address)
# target_link_options(fibar PRIVATE -fsanitize=address)

target_include_directories(fibar PUBLIC include)
rclcpp_components_register_nodes(fibar "event_image_reconstruction_fibar::Fibar")

#
# -------- fibar node
#
add_executable(fibar_node src/fibar_node.cpp)
target_link_libraries(fibar_node PUBLIC fibar rclcpp::rclcpp)
target_include_directories(fibar_node PRIVATE include)

#
# -------- performance test
#
add_executable(performance_test src/performance_test.cpp)
target_include_directories(performance_test PUBLIC include)
target_link_libraries(performance_test
  fibar
  rclcpp::rclcpp
  rosbag2_transport::rosbag2_transport
  ${sensor_msgs_TARGETS})

#
# -------- bag_to_frames
#
add_executable(bag_to_frames src/bag_to_frames_main.cpp)
target_include_directories(bag_to_frames PUBLIC include)

target_link_libraries(bag_to_frames
  cv_bridge::cv_bridge
  ${event_camera_codecs_TARGETS}
  ${event_camera_msgs_TARGETS}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  ${sensor_msgs_TARGETS}
  fibar_lib::fibar_lib
  opencv_core opencv_imgcodecs)

# the nodes must go into the paroject specific lib directory or else
# the launch file will not find it

install(TARGETS
  fibar_node
  bag_to_frames
  performance_test
  DESTINATION lib/${PROJECT_NAME}/)

install(TARGETS
    fibar
    DESTINATION lib)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_black REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_black()
  ament_lint_cmake()
  ament_xmllint()
endif()

ament_package()
