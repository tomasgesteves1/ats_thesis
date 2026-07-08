cmake_minimum_required(VERSION 3.5)
project(bringup)

find_package(ament_cmake REQUIRED)

# Instala as pastas dentro de share/bringup/
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
