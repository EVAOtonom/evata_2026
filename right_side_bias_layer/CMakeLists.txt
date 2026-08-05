cmake_minimum_required(VERSION 3.8)
project(right_side_bias_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(right_side_bias_layer_core SHARED
  src/right_side_bias_layer.cpp
)

target_compile_features(right_side_bias_layer_core PUBLIC cxx_std_17)
target_include_directories(right_side_bias_layer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(right_side_bias_layer_core
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(nav2_costmap_2d right_side_bias_layer.xml)

install(TARGETS right_side_bias_layer_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES right_side_bias_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(right_side_bias_layer_core)
ament_export_dependencies(geometry_msgs nav2_costmap_2d pluginlib rclcpp)
ament_package()
