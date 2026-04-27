cmake_minimum_required(VERSION 3.5)
project(ats_description)

find_package(ament_cmake REQUIRED)

# Instala as pastas cruciais para que o launch file as encontre
install(DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()