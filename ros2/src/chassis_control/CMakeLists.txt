cmake_minimum_required(VERSION 3.5)
project(chassis_control)

find_package(ament_cmake REQUIRED)

# Install Python module (importable as chassis_control.ps100_driver)
install(DIRECTORY ${PROJECT_NAME}
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
)

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/chassis_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
