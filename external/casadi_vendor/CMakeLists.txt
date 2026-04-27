cmake_minimum_required(VERSION 3.5)
project(casadi_vendor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(casadi_DESTDIR ${CMAKE_CURRENT_BINARY_DIR}/casadi_install)

include(ExternalProject)
ExternalProject_Add(casadi
  LOG_CONFIGURE ON
  LOG_INSTALL ON
  LOG_BUILD ON
  LOG_OUTPUT_ON_FAILURE ON
  GIT_REPOSITORY https://github.com/casadi/casadi
  GIT_TAG 3.5.5
  GIT_SHALLOW TRUE
  BUILD_IN_SOURCE TRUE
  INSTALL_DIR ${casadi_DESTDIR}
  CONFIGURE_COMMAND ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DWITH_COMMON=ON -DWITH_IPOPT=ON -DWITH_THREAD=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON
)

file(GLOB SHARED_LIBS
  "${casadi_DESTDIR}/lib/*.so"
)

set(SHORT_LIBS "")

foreach(SHARED_LIB ${SHARED_LIBS})
  string(REPLACE ${casadi_DESTDIR}/lib/ "" SHORT_LIB ${SHARED_LIB})
  list(APPEND SHORT_LIBS ${SHORT_LIB})
endforeach()

ament_export_include_directories(include)
ament_export_libraries(${SHORT_LIBS})

# Move the cmake configurations such that mavsdk can be found automatically
install(DIRECTORY ${casadi_DESTDIR}/lib/cmake/casadi/ 
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY ${casadi_DESTDIR}/include/
  DESTINATION include
)

install(DIRECTORY ${casadi_DESTDIR}/lib/
  DESTINATION lib
)

ament_package()
