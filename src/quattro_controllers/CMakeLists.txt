cmake_minimum_required(VERSION 3.8)
project(quattro_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(mit_trajectory_controller SHARED
  src/mit_trajectory_controller.cpp)
target_compile_features(mit_trajectory_controller PUBLIC cxx_std_17)
target_include_directories(mit_trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mit_trajectory_controller
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  realtime_tools trajectory_msgs)

pluginlib_export_plugin_description_file(
  controller_interface quattro_controllers.xml)

install(TARGETS mit_trajectory_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES quattro_controllers.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  controller_interface hardware_interface rclcpp trajectory_msgs)
ament_package()
