cmake_minimum_required(VERSION 3.8)
project(wu25)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include/wu25)
include_directories(sh2)
# include_directories("include/wu25")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)


add_library(bno08x src/drivers/bno08x.cpp include/wu25/bno08x.hpp)

add_library(sh2lib sh2/euler.c sh2/sh2_SensorValue.c sh2/sh2.c sh2/sh2_util.c sh2/shtp.c)

add_executable(talker src/publisher_lambda_function.cpp)
add_executable(listener src/subscriber_lambda_function.cpp)
add_executable(bno_node src/bno_node.cpp include/wu25/bno_node.hpp include/wu25/bno08x.hpp include/wu25/spi.hpp src/drivers/bno08x.cpp src/drivers/spi.cpp sh2/euler.c sh2/sh2_SensorValue.c sh2/sh2.c sh2/sh2_util.c sh2/shtp.c)

target_link_libraries(talker Eigen3::Eigen libwiringPi.so libgpiod.so)
target_link_libraries(bno_node sh2lib bno08x libwiringPi.so libgpiod.so)

target_include_directories(bno_node
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(talker rclcpp std_msgs tf2 tf2_ros)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(bno_node rclcpp std_msgs)

install(TARGETS
  talker
  listener
  sh2lib
  bno_node
  DESTINATION lib/${PROJECT_NAME})

  ament_python_install_package(scripts)

  install(DIRECTORY
   launch
   DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
