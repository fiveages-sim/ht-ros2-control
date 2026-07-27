cmake_minimum_required(VERSION 3.12)
project(panthera_ros2_control)

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
# HI never uses LCM motor_msg publish; skip vendored LCM (+ glib) entirely.
set(HIGHTORQUE_MOTOR_ENABLE_LCM OFF CACHE BOOL "Build/link vendored LCM for motor_msg publish" FORCE)

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

pluginlib_export_plugin_description_file(hardware_interface panthera_ros2_control.xml)

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
  FILES panthera_ros2_control.xml
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
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
