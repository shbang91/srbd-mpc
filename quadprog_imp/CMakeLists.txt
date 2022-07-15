cmake_minimum_required(VERSION 3.5.1)

project(srbd-mpc)

##=============================================================================
## CMake Flags
##=============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS on)
set(CXX_DISABLE_WERROR True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

##=============================================================================
## Include Directory
##=============================================================================
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("/usr/local/include")
link_directories("/usr/local/lib")

find_package(Eigen 3.3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

add_subdirectory(goldfarb)
add_subdirectory(mpc)

add_executable(main main.cpp)
target_link_libraries(main mpc goldfarb)

set(THIS_COM "${PROJECT_SOURCE_DIR}/")
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake
    ${PROJECT_SOURCE_DIR}/configuration.hpp)
