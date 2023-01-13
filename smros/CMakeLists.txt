cmake_minimum_required(VERSION 3.0.2)
project(smros)
#SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        pcl_conversions
        rangenet_lib
        cv_bridge
        image_transport
        )
find_package(PCL REQUIRED)
find_package(CUDA REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP)
find_package(Boost REQUIRED COMPONENTS filesystem system thread date_time timer chrono serialization)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        ndt_omp
        CATKIN_DEPENDS
        rangenet_lib
        DEPENDS
        Boost
)

include_directories(
        # include
        src
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(kittireader
        src/kittireader.cpp
        )
target_link_libraries(kittireader ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(perception
        src/perception.cpp
        src/RangenetAPI.cpp
        )
target_link_libraries(perception ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(localization
        src/backendopt.cpp
        src/localization.cpp
        src/semanticmap.cpp
        src/Scancontext/Scancontext.cpp)
target_link_libraries(localization ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)