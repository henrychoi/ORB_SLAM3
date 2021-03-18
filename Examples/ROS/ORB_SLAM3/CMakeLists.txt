cmake_minimum_required(VERSION 3.5)
project(orb_slam3_ros2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge)
find_package(Pangolin REQUIRED PATHS /usr/local/lib)
if(NOT Pangolin_FOUND)
   message(FATAL_ERROR "pangolin not found.")
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(ORB_SLAM3 REQUIRED)


# Node for stereo camera
add_executable(Stereo src/ros_stereo.cc)
ament_target_dependencies(Stereo rclcpp sensor_msgs Eigen3 Pangolin ORB_SLAM3 cv_bridge)

# Node for stereo camera with IMU
add_executable(Imu_stereo src/ros_stereo_inertial.cc)
ament_target_dependencies(Imu_stereo rclcpp sensor_msgs Eigen3 Pangolin ORB_SLAM3 cv_bridge)


# Install 
install(
  TARGETS Imu_stereo Stereo 
  DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
