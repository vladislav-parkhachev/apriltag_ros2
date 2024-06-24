cmake_minimum_required(VERSION 3.8)
project(apriltag_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)


find_package(OpenCV REQUIRED)

find_package(apriltag REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

get_target_property(apriltag_INCLUDE_DIRS apriltag::apriltag INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(apriltag_LIBRARIES apriltag::apriltag INTERFACE_LINK_LIBRARIES)

set(srv_files
  "srv/AnalyzeSingleImage.srv"
)

set(msg_files
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_executable(apriltag_ros2_single_image_server_node src/apriltag_ros2_single_image_server_node.cpp)
ament_target_dependencies(apriltag_ros2_single_image_server_node 
  rclcpp 
  std_msgs 
  geometry_msgs
  sensor_msgs
  apriltag 
  tf2_ros 
  cv_bridge
  OpenCV)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(apriltag_ros2_single_image_server_node "${cpp_typesupport_target}")

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

install(TARGETS
apriltag_ros2_single_image_server_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
