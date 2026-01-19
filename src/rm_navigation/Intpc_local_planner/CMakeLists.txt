cmake_minimum_required(VERSION 3.8)
project(intpc_local_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
  # 或者移除-Werror的默认设置：
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(nav2_common REQUIRED)
find_package(ament_cmake REQUIRED)

# Find g2o (system library)
find_package(g2o REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(dwb_critics REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
#find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teb_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

nav2_package()

# Include directories
include_directories(
  include
  ${g2o_INCLUDE_DIRS}
)

# Build the plugin library
add_library(${PROJECT_NAME}_plugin SHARED
  src/intpc_local_planner.cpp
  src/intpc_local_planner_ros.cpp
  src/intpc_global_planner_ros.cpp
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}_plugin
    costmap_converter
    dwb_critics
    nav2_core
    nav2_costmap_2d
    nav2_util
    geometry_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    pluginlib
    teb_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    builtin_interfaces
)

# Link g2o libraries
target_link_libraries(${PROJECT_NAME}_plugin
  ${g2o_LIBRARIES}
)


# Install library
install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install Python scripts
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Export library
ament_export_libraries(${PROJECT_NAME}_plugin)

# Export dependencies
ament_export_dependencies(
    costmap_converter
    dwb_critics
    nav2_core
    nav2_costmap_2d
    nav2_util
    geometry_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    pluginlib
    teb_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    builtin_interfaces
)

# Export plugin
pluginlib_export_plugin_description_file(nav2_core intpc_local_planner_plugin.xml)

# Install plugin description file
install(FILES intpc_local_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME})

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()