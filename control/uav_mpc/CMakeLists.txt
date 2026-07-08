cmake_minimum_required(VERSION 3.8)
project(uav_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# ACADOS (Installed globally)
set(ACADOS_DIR "/home/tomas/acados")
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
add_library(uav_acados_solver STATIC
  acados_solver/acados_solver_uav_tethered.c
  acados_solver/uav_tethered_model/uav_tethered_expl_ode_fun.c
  acados_solver/uav_tethered_model/uav_tethered_expl_vde_forw.c
  acados_solver/uav_tethered_model/uav_tethered_expl_vde_adj.c
  acados_solver/uav_tethered_constraints/uav_tethered_constr_h_fun_jac_uxt_zt.c
  acados_solver/uav_tethered_constraints/uav_tethered_constr_h_fun.c
  acados_solver/uav_tethered_constraints/uav_tethered_constr_h_e_fun_jac_uxt_zt.c
  acados_solver/uav_tethered_constraints/uav_tethered_constr_h_e_fun.c
)
target_link_libraries(uav_acados_solver acados hpipm blasfeo)

# ROS 2 Node
add_executable(uav_mpc_node 
  src/uav_mpc_main.cpp
  src/uav_mpc_node.cpp
  src/uav_mpc_pipeline.cpp
  src/uav_mpc_trajectory.cpp
  src/uav_mpc_kinematics.cpp
  src/uav_mpc_tether.cpp
)
target_include_directories(uav_mpc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(uav_mpc_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  ros_gz_interfaces
  px4_msgs
  visualization_msgs
  tf2_ros
)
target_link_libraries(uav_mpc_node uav_acados_solver Eigen3::Eigen)

install(TARGETS uav_mpc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
