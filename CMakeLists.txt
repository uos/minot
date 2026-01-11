cmake_minimum_required(VERSION 3.5)
project(minot)

find_package(ament_cmake REQUIRED)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

include(r2r_cargo.cmake)

r2r_cargo(rcl rcl_action sensor_msgs nav_msgs)

install(PROGRAMS
  ${CMAKE_SOURCE_DIR}/target/colcon/${PROJECT_NAME}${SUFFIX}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

