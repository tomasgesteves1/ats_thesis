cmake_minimum_required(VERSION 3.5)
project(ats_gazebo)

find_package(ament_cmake REQUIRED)

# Instala as pastas dentro de share/ats_gazebo/
install(DIRECTORY models worlds resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
