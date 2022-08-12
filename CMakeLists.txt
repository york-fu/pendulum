cmake_minimum_required(VERSION 3.0.0)
project(pendulum VERSION 0.1.0)

include(CTest)
enable_testing()

set(CMAKE_BUILD_TYPE "Release")

include_directories(./include)
include_directories(./lcm_types)

add_subdirectory(lib/elmo_driver)
add_subdirectory(lcmtypes)
add_subdirectory(src)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
