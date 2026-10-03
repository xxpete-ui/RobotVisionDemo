cmake_minimum_required(VERSION 3.8)
project(robot_vision_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(
  ROBOTVISION_SOURCE_DIR ""
  CACHE PATH "Path to the RobotVisionDemo source repository"
)

if(NOT EXISTS "${ROBOTVISION_SOURCE_DIR}/src/CoordinateTransform.cpp")
  message(FATAL_ERROR
    "Set ROBOTVISION_SOURCE_DIR to your RobotVisionDemo repository")
endif()

add_library(robot_vision_geometry STATIC
  "${ROBOTVISION_SOURCE_DIR}/src/CoordinateTransform.cpp"
  "${ROBOTVISION_SOURCE_DIR}/src/TransformUtils.cpp"
  "${ROBOTVISION_SOURCE_DIR}/src/Logger.cpp"
)

target_include_directories(robot_vision_geometry PUBLIC
  "${ROBOTVISION_SOURCE_DIR}/include"
)

target_compile_features(robot_vision_geometry PUBLIC cxx_std_17)

add_executable(target_listener src/target_listener.cpp)
target_include_directories(target_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(target_listener PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  target_listener
  "rclcpp"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
)

target_link_libraries(target_listener robot_vision_geometry)

add_executable(
  dynamic_transform_demo
  src/dynamic_transform_demo.cpp
)

target_compile_features(
  dynamic_transform_demo PUBLIC cxx_std_17
)

ament_target_dependencies(
  dynamic_transform_demo
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(
  target_publisher
  src/target_publisher.cpp
)

target_compile_features(
  target_publisher PUBLIC cxx_std_17
)

ament_target_dependencies(
  target_publisher
  rclcpp
  geometry_msgs
)

install(TARGETS
  target_listener
  dynamic_transform_demo
  target_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
