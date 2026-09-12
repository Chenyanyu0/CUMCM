
cmake_minimum_required(VERSION 3.10)
project(radio_robot_cpp14_final_optimized)

set(CMAKE_CXX_STANDARD 14)
find_package(CURL REQUIRED)

include_directories(include)

add_executable(robot
src/main.cpp
src/api.cpp
src/geometry.cpp
src/localization.cpp
src/search.cpp
src/planner.cpp
src/logger.cpp
)

target_link_libraries(robot CURL::libcurl)
