cmake_minimum_required(VERSION 3.5)
project(task_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_executor REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(task_control SHARED
  src/material_manager.cpp
  src/mission_scheduler.cpp
  src/task_manager.cpp
)

target_compile_features(task_control PUBLIC cxx_std_17)
target_include_directories(task_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(task_control
  robot_executor::robot_executor
  yaml-cpp
)

ament_target_dependencies(task_control
  rclcpp
  robot_interfaces
  std_srvs
  geometry_msgs
)

add_executable(task_manager_node
  src/task_manager_node.cpp
)

target_compile_features(task_manager_node PUBLIC cxx_std_17)
target_include_directories(task_manager_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(task_manager_node task_control)

install(TARGETS task_control task_manager_node
  EXPORT export_task_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_task_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
