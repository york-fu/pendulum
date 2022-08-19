cmake_minimum_required(VERSION 3.0.0)
project(pendulum VERSION 0.1.0)

set(CMAKE_BUILD_TYPE "Release")

find_package(drake CONFIG REQUIRED)
find_package(lcm CONFIG REQUIRED)

include_directories(../common)
include_directories(./)

set(SRCS
  ../common/utils.cpp
  ../common/force_disturber.cc
  ../common/lcm_publish.cc
  ../common/actuators_interface.cc
  ./make_pendulum_plant.cc
  ./hardware_plant.cc
  ./lqrfeedforward.cc
)
add_executable(${PROJECT_NAME}_lqr ${SRCS} ./lqr_control.cc)
target_link_libraries(${PROJECT_NAME}_lqr pthread gflags drake::drake elmo)
lcm_target_link_libraries(${PROJECT_NAME}_lqr lcm lcm-types-cpp)
