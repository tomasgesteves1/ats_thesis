cmake_minimum_required(VERSION 3.5)
project(mission_manager_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/UserParam.msg"
  "msg/MissionDefinition.msg"
)

set(SRV_FILES
  "srv/ListMissions.srv"
  "srv/StartMission.srv"
  "srv/StopMission.srv"
  "srv/GetMissionStatus.srv"
  "srv/StartSimulation.srv"
  "srv/StopSimulation.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
