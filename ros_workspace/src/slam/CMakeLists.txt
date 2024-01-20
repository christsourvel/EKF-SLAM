cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseMsg.msg"
  "msg/Perception2Slam.msg"
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Add localization executable
add_executable(localization src/localization.cpp)
target_compile_features(localization PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(localization rclcpp Eigen3 geometry_msgs)
rosidl_target_interfaces(localization ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add slam_handler executable
add_executable(slam_handler src/slam_handler.cpp)
target_compile_features(slam_handler PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(slam_handler rclcpp Eigen3 geometry_msgs)
rosidl_target_interfaces(slam_handler ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add slam_from_file executable
add_executable(slam_from_file src/slam_from_file.cpp)
target_compile_features(slam_from_file PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(slam_from_file rclcpp Eigen3 geometry_msgs)
rosidl_target_interfaces(slam_from_file ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add velocity_publisher executable
add_executable(velocity_publisher src/velocity_publisher.cpp)
ament_target_dependencies(velocity_publisher rclcpp geometry_msgs)

# Install executables
install(TARGETS
  localization
  slam_handler
  slam_from_file
  velocity_publisher
  DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
