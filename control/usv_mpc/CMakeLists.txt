cmake_minimum_required(VERSION 3.8)
project(usv_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ACADOS (Installed globally)
set(ACADOS_DIR $ENV{ACADOS_SOURCE_DIR})
include_directories(
  include
  acados_solver
  ${ACADOS_DIR}/include
  ${ACADOS_DIR}/include/acados
  ${ACADOS_DIR}/include/blasfeo/include
  ${ACADOS_DIR}/include/hpipm/include
)
link_directories(
  ${ACADOS_DIR}/lib
)

# Build ACADOS generated code as a static library
add_library(usv_acados_solver STATIC
  acados_solver/acados_solver_usv_kinematic.c
  acados_solver/usv_kinematic_model/usv_kinematic_expl_ode_fun.c
  acados_solver/usv_kinematic_model/usv_kinematic_expl_vde_forw.c
  acados_solver/usv_kinematic_model/usv_kinematic_expl_vde_adj.c
)
target_link_libraries(usv_acados_solver acados hpipm blasfeo)

# ROS 2 Node
add_executable(usv_mpc_node 
  src/usv_mpc_main.cpp
  src/usv_mpc_node.cpp
  src/usv_mpc_pipeline.cpp
)
target_include_directories(usv_mpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(usv_mpc_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(usv_mpc_node usv_acados_solver)

install(TARGETS usv_mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
