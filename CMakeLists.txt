cmake_minimum_required(VERSION 3.0.2)
project(smros)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        pcl_conversions
        rangenet_lib
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

message("${PCL_INCLUDE_DIRS}")
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

add_library(scancontext
        include/Scancontext/Scancontext.cpp)

add_executable(smros
        src/core/main.cpp
        src/core/RangenetAPI.cpp
        src/core/sumaslam.cpp
        src/core/vertexmap.cpp
        src/core/surfelmap.cpp
#        src/core/PointIndex.cpp
        src/core/backendopt.cpp
        src/core/timestamp.cpp
#        robovision
        include/rv/CompositeParameter.cpp
        include/rv/FileUtil.cpp
        include/rv/Laserscan.cpp
        include/rv/Math.cpp
        include/rv/Parameter.cpp
        include/rv/ParameterList.cpp
        include/rv/ParameterListIterator.cpp
        include/rv/PrimitiveParameters.cpp
        include/rv/Random.cpp
        include/rv/RangeParameter.cpp
        include/rv/string_utils.cpp
        include/rv/Stopwatch.cpp
        include/rv/transform.cpp
        include/rv/XmlDocument.cpp
        include/rv/XmlNode.cpp
        include/rv/yxml.c
        )
target_link_libraries(smros ${catkin_LIBRARIES} ${PCL_LIBRARIES} ndt_omp ${G2O_LIBRARIES} ${Boost_LIBRARIES} gtsam scancontext)