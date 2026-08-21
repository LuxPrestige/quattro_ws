cmake_minimum_required(VERSION 3.8)
project(quattro_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gim6010_driver REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(quattro_hardware SHARED
  src/joint_transform.cpp
  src/quattro_system.cpp)
target_compile_features(quattro_hardware PUBLIC cxx_std_17)
target_include_directories(quattro_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(quattro_hardware PUBLIC gim6010_driver::gim6010_driver)
ament_target_dependencies(quattro_hardware PUBLIC
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface quattro_hardware.xml)

# calibration_gui: standalone Qt tool, deliberately independent of
# ros2_control/controller_manager (docs/packages/quattro_hardware.md
# section 5) -- it talks to gim6010_driver directly.
find_package(Qt5 COMPONENTS Widgets QUIET)
find_package(yaml-cpp QUIET)
if(Qt5_FOUND AND yaml-cpp_FOUND)
  set(CMAKE_AUTOMOC ON)
  add_executable(calibration_gui src/calibration_gui.cpp)
  target_compile_features(calibration_gui PUBLIC cxx_std_17)
  target_link_libraries(calibration_gui
    quattro_hardware
    gim6010_driver::gim6010_driver
    Qt5::Widgets
    yaml-cpp::yaml-cpp)
  install(TARGETS calibration_gui DESTINATION lib/${PROJECT_NAME})
  add_executable(direct_position_tuning_gui src/direct_position_tuning_gui.cpp)
  target_compile_features(direct_position_tuning_gui PUBLIC cxx_std_17)
  target_link_libraries(direct_position_tuning_gui
    quattro_hardware
    gim6010_driver::gim6010_driver
    Qt5::Widgets
    yaml-cpp::yaml-cpp)
  install(TARGETS direct_position_tuning_gui DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "Qt5 Widgets and/or yaml-cpp not found -- skipping hardware GUI tools. "
    "Install qtbase5-dev and libyaml-cpp-dev to build it.")
endif()

install(
  TARGETS quattro_hardware
  EXPORT export_quattro_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES quattro_hardware.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_transform test/test_joint_transform.cpp)
  target_link_libraries(test_joint_transform quattro_hardware)

  ament_add_gtest(test_pluginlib_export test/test_pluginlib_export.cpp)
  target_link_libraries(test_pluginlib_export quattro_hardware)
  ament_target_dependencies(test_pluginlib_export pluginlib hardware_interface)
endif()

ament_export_include_directories(include)
ament_export_libraries(quattro_hardware)
ament_export_targets(export_quattro_hardware)
ament_export_dependencies(gim6010_driver hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
