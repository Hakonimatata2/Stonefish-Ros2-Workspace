cmake_minimum_required(VERSION 3.8)
project(sensors)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

file(GLOB_RECURSE MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")
if(NOT MSG_FILES)
  message(FATAL_ERROR "No .msg files found in msg/")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs sensor_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
