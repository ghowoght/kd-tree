cmake_minimum_required(VERSION 3.0.4)
project(kdtree)

find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)
target_link_libraries(main ${PCL_LIBRARIES})