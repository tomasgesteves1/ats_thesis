cmake_minimum_required(VERSION 3.5)
project(ats_bringup)

find_package(ament_cmake REQUIRED)

# Instala as pastas dentro de share/ats_bringup/
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
