cmake_minimum_required(VERSION 3.0.2)
project(smros)
SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        pcl_conversions
        rangenet_lib
        smros_msgs
)
find_package(PCL REQUIRED)
find_package(CUDA REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP)
find_package(Boost REQUIRED COMPONENTS filesystem system thread date_time timer chrono serialization)
find_package(GTSAM REQUIRED)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS
        include
  LIBRARIES
        ndt_omp
  CATKIN_DEPENDS
        rangenet_lib
  DEPENDS
        Boost
)

include_directories(
# include
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
)

add_library(ndt_omp
        src/pclomp/voxel_grid_covariance_omp.cpp
        src/pclomp/gicp_omp.cpp
        src/pclomp/ndt_omp.cpp
        )

#add_library(scancontext
#        include/Scancontext/Scancontext.cpp)

#add_executable(smros
#        src/core/main.cpp
#        src/core/RangenetAPI.cpp
#        src/core/sumaslam.cpp
#        src/core/vertexmap.cpp
#        src/core/surfelmap.cpp
##        src/core/PointIndex.cpp
#        src/core/backendopt.cpp
#        src/core/timestamp.cpp
#        include/Scancontext/Scancontext.cpp
#        )
#target_link_libraries(smros ${catkin_LIBRARIES} ${PCL_LIBRARIES} ndt_omp ${G2O_LIBRARIES} ${Boost_LIBRARIES} gtsam)

add_executable(lasereader
        src/rosnode/lasereader.cpp
        src/core/KittiReader.cpp
        )
target_link_libraries(lasereader ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(laserprocess
        src/rosnode/laserprocess.cpp
        src/core/Preprocess.cpp
        src/core/RangenetAPI.cpp
        src/core/vertexmap.cpp
        )
target_link_libraries(laserprocess ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(laserodometry
        src/rosnode/laserodometry.cpp
        src/core/Odometry.cpp
        src/core/backendopt.cpp
        )
target_link_libraries(laserodometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

add_executable(laserloop
        src/rosnode/laserloop.cpp
        src/core/LoopClosureDetection.cpp
        include/Scancontext/Scancontext.cpp
        )
target_link_libraries(laserloop ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(laserloop ${catkin_EXPORTED_TARGETS})

add_executable(lasermap
        src/rosnode/lasermap.cpp
        src/core/SemanticMap.cpp
        src/core/backendopt.cpp
        )
target_link_libraries(lasermap ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)
add_dependencies(lasermap ${catkin_EXPORTED_TARGETS})