cmake_minimum_required(VERSION 3.5)
project(gazebo)

find_package(ament_cmake REQUIRED)

# Instala as pastas dentro de share/gazebo/
install(DIRECTORY models worlds resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
