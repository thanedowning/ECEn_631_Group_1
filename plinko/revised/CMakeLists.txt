cmake_minimum_required(VERSION 2.8)
project(plinko)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(plinko plinko.cpp)
target_link_libraries(plinko ${OpenCV_LIBS})
