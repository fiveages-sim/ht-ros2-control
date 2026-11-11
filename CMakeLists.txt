cmake_minimum_required(VERSION 3.16)
project(ht_ros2_control)

if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(MOTOR_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/motor_cpp)
set(PANTHERA_SDK_BUILD_EXAMPLES OFF CACHE BOOL "Build vendored Panthera SDK examples" FORCE)

add_subdirectory(
  ${MOTOR_CPP_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/motor_cpp
)

add_library(${PROJECT_NAME} SHARED
  src/panthera_hardware_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} hightorque_motor)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface ht_ros2_control.xml)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Motor bus YAML configs (Panthera.yaml / PantheraDual.yaml).
install(
  DIRECTORY
    ${MOTOR_CPP_DIR}/robot_param
  DESTINATION share/${PROJECT_NAME}/external/motor_cpp
)

install(
  FILES ht_ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  add_executable(feedback_snapshot_oracle tests/feedback_snapshot_oracle.cpp)
  target_compile_features(feedback_snapshot_oracle PRIVATE cxx_std_17)
  target_include_directories(feedback_snapshot_oracle PRIVATE include)
  target_link_libraries(feedback_snapshot_oracle hightorque_motor)
  add_test(NAME feedback_snapshot_oracle COMMAND feedback_snapshot_oracle)
  set_tests_properties(feedback_snapshot_oracle PROPERTIES TIMEOUT 30)
  add_executable(packet_oracle tests/packet_oracle.cpp)
  target_compile_features(packet_oracle PRIVATE cxx_std_17)
  target_link_libraries(packet_oracle hightorque_motor)
  add_test(NAME packet_oracle COMMAND packet_oracle ${CMAKE_CURRENT_BINARY_DIR}/MIT-frames.bin)
  set_tests_properties(packet_oracle PROPERTIES TIMEOUT 30)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
