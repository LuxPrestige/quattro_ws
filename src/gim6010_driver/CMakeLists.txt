cmake_minimum_required(VERSION 3.8)
project(gim6010_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(gim6010_driver SHARED
  src/can_socket.cpp
  src/can_error.cpp
  src/can_simple_messages.cpp
  src/mit_protocol.cpp
  src/gim6010_motor.cpp
  src/motor_manager.cpp)
target_compile_features(gim6010_driver PUBLIC cxx_std_17)
target_include_directories(gim6010_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(gim6010_diagnostic src/gim6010_diagnostic.cpp)
target_compile_features(gim6010_diagnostic PUBLIC cxx_std_17)
target_link_libraries(gim6010_diagnostic gim6010_driver)

install(
  TARGETS gim6010_driver
  EXPORT export_gim6010_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS gim6010_diagnostic DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_can_simple_messages test/test_can_simple_messages.cpp)
  target_link_libraries(test_can_simple_messages gim6010_driver)

  ament_add_gtest(test_mit_protocol test/test_mit_protocol.cpp)
  target_link_libraries(test_mit_protocol gim6010_driver)

  ament_add_gtest(test_can_error test/test_can_error.cpp)
  target_link_libraries(test_can_error gim6010_driver)

  ament_add_gtest(test_motor_manager test/test_motor_manager.cpp)
  target_link_libraries(test_motor_manager gim6010_driver)
endif()

ament_export_include_directories(include)
ament_export_libraries(gim6010_driver)
ament_export_targets(export_gim6010_driver)
ament_package()
