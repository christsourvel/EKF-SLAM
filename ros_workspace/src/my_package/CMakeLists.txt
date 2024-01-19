cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add my_node executable
add_executable(my_node src/my_node.cpp)
target_include_directories(my_node PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)
# Link my_node executable with rclcpp and geometry_msgs
ament_target_dependencies(my_node rclcpp Eigen3 geometry_msgs)

# Add velocity_publisher executable
add_executable(velocity_publisher src/velocity_publisher.cpp)
# Link velocity_publisher executable with rclcpp and geometry_msgs
ament_target_dependencies(velocity_publisher rclcpp geometry_msgs)

# Install executables
install(TARGETS
  my_node
  velocity_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
