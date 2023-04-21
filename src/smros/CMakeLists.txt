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
find_package(Ceres REQUIRED)

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS
        rangenet_lib
        DEPENDS
        Boost
)

include_directories(
        # include
        include
        third_party
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(third_party)

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
        src/surfel.cpp
        src/pose_estimate.cpp
        src/utils.cpp)
target_link_libraries(localization ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam ndt_omp scan_context Ceres::ceres)

add_executable(test_rangenet
        src/test_rangenet.cpp
        src/RangenetAPI.cpp
        )
target_link_libraries(test_rangenet ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})