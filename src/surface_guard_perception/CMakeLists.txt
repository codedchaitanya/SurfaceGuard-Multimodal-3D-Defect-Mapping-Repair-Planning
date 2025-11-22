cmake_minimum_required(VERSION 3.8)
project(surface_guard_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(surface_guard_interfaces REQUIRED)

# Find PCL (Geometric Processing)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation extract_indices search kdtree)
include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# --- Build C++ Node ---
add_executable(wall_processor_node src/wall_processor_node.cpp)
ament_target_dependencies(wall_processor_node 
  rclcpp 
  sensor_msgs 
  pcl_conversions 
  surface_guard_interfaces
)
# Link PCL libraries explicitly
target_link_libraries(wall_processor_node ${PCL_LIBRARIES})

# --- Install Python Node ---
install(PROGRAMS
  scripts/crack_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install Config & Header ---
install(DIRECTORY config models include/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()