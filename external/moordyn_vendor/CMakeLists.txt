cmake_minimum_required(VERSION 3.8)
project(moordyn_vendor)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

set(moordyn_DESTDIR ${CMAKE_CURRENT_BINARY_DIR}/moordyn_install)

ExternalProject_Add(moordyn_external
  GIT_REPOSITORY https://github.com/FloatingArrayDesign/MoorDyn.git
  GIT_TAG v2.6.1
  GIT_SHALLOW TRUE
  PREFIX moordyn_build
  INSTALL_DIR ${moordyn_DESTDIR}
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DCMAKE_BUILD_TYPE=Release
    -DPYTHON_WRAPPER=OFF
    -DEXTERNAL_EIGEN=ON
    -DFORTRAN_WRAPPER=OFF
    -DMATLAB_WRAPPER=OFF
)

install(DIRECTORY ${moordyn_DESTDIR}/include/
  DESTINATION include
)

install(DIRECTORY ${moordyn_DESTDIR}/lib/
  DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(moordyn)

ament_package()

