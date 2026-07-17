cmake_minimum_required(VERSION 3.5)
project(robot_interfaces)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Task.msg"
  "msg/TaskResult.msg"
  "srv/ExecuteTask.srv"
  DEPENDENCIES
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
