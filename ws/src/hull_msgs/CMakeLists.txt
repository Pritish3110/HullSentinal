cmake_minimum_required(VERSION 3.10)
project(hull_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DefectDetection.msg"
  "msg/TaskItem.msg"
  "msg/TaskQueue.msg"
  "msg/BeliefUpdate.msg"
  "msg/RobotStatus.msg"
  "msg/MissionStatus.msg"
  "srv/TaskComplete.srv"
  "srv/RescanRequest.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
