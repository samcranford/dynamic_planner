cmake_minimum_required(VERSION 3.5)
project(dynamic_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wuninitialized)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(planner_lib STATIC src/planner.cpp)
target_include_directories(planner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(planner_lib nav_msgs)

add_executable(planner_executable src/main.cpp)

target_link_libraries(planner_executable PRIVATE planner_lib)

ament_package()