cmake_minimum_required(VERSION 3.8)
project(apriltag_ros2_image_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(apriltag_ros2_interfaces REQUIRED)   
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

add_executable(apriltag_ros2_single_image_server_node src/apriltag_ros2_single_image_server_node.cpp)
ament_target_dependencies(apriltag_ros2_single_image_server_node 
  rclcpp 
  std_msgs
  # apriltag_ros2_interfaces
  cv_bridge
  OpenCV
  )  

target_include_directories(apriltag_ros2_single_image_server_node PUBLIC apriltag)
target_link_libraries(apriltag_ros2_single_image_server_node apriltag)

install(TARGETS
  apriltag_ros2_single_image_server_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
