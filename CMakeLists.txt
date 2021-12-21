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

catkin_package(
  INCLUDE_DIRS
        src
#  LIBRARIES smros
  CATKIN_DEPENDS
        rangenet_lib
#  DEPENDS system_lib
)

message("${PCL_INCLUDE_DIRS}")
include_directories(
# include
    src
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_executable(smros
        src/core/main.cpp
        src/core/RangenetAPI.cpp
        src/core/SumaSLAM.cpp
        src/core/Frame.cpp
        src/core/SurfelMap.cpp
#        robovision
        src/rv/CompositeParameter.cpp
        src/rv/FileUtil.cpp
        src/rv/Laserscan.cpp
        src/rv/Math.cpp
        src/rv/Parameter.cpp
        src/rv/ParameterList.cpp
        src/rv/ParameterListIterator.cpp
        src/rv/PrimitiveParameters.cpp
        src/rv/Random.cpp
        src/rv/RangeParameter.cpp
        src/rv/string_utils.cpp
        src/rv/Stopwatch.cpp
        src/rv/transform.cpp
        src/rv/XmlDocument.cpp
        src/rv/XmlNode.cpp
        src/rv/yxml.c
        )
target_link_libraries(smros ${catkin_LIBRARIES} ${PCL_LIBRARIES})