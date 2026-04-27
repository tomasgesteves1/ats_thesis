cmake_minimum_required(VERSION 3.5)
project(micro_xrce_vendor)

# Default to C17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
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

set(micro_xrce_DESTDIR ${CMAKE_CURRENT_BINARY_DIR}/micro_xrce_install)

include(ProcessorCount)
ProcessorCount(N)

include(ExternalProject)
ExternalProject_Add(micro_xrce
  LOG_CONFIGURE ON
  LOG_INSTALL ON
  LOG_BUILD ON
  LOG_OUTPUT_ON_FAILURE ON
  GIT_REPOSITORY https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  GIT_TAG v3.0.1
  GIT_SHALLOW TRUE
  BUILD_IN_SOURCE FALSE
  SOURCE_DIR           ${CMAKE_BINARY_DIR}/micro_xrce-prefix/micro_xrce
  INSTALL_DIR          ${micro_xrce_DESTDIR}
  CONFIGURE_COMMAND    cmake .. -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF
  PATCH_COMMAND        mkdir -p build
  BINARY_DIR           ${CMAKE_BINARY_DIR}/micro_xrce-prefix/micro_xrce/build
  BUILD_DIR            ${CMAKE_BINARY_DIR}/micro_xrce-prefix/micro_xrce/build
  BUILD_COMMAND        make -j ${N}
)

# Move the executable to installation directory
install(DIRECTORY ${micro_xrce_DESTDIR}/bin/
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
