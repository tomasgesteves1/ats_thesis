cmake_minimum_required(VERSION 3.5)
project(ats_control)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# add executable
add_executable(usv_controller src/usv_controller.cpp)
target_include_directories(usv_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(usv_controller rclcpp geometry_msgs sensor_msgs std_msgs)

# install
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# O DESTINATION correto é lib/${PROJECT_NAME}
install(TARGETS
  usv_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
