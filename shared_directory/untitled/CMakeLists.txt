cmake_minimum_required(VERSION 3.24)
project(untitled)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
        # include
        ${PCL_INCLUDE_DIRS}
)

add_executable(untitled main.cpp point2plane.cpp point2plane.h lidar_cost_function.h nonliner.cpp)
target_link_libraries(untitled ${PCL_LIBRARIES} Ceres::ceres)
