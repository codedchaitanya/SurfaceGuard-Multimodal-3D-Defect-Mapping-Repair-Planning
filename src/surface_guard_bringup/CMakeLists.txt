cmake_minimum_required(VERSION 3.8)
project(surface_guard_bringup)

find_package(ament_cmake REQUIRED)

# Install Launch Files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz Configs
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()