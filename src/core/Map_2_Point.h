//
// Created by tongda on 2022/1/2.
//

#ifndef SRC_MAP_2_POINT_H
#define SRC_MAP_2_POINT_H

#include <rv/ParameterList.h>
#include <memory>
#include <Eigen/Dense>
#include "surfel.h"
#include <cmath>

class Map_2_Point {
private:
    int height_, width_;
    float fov_up_, fov_down_, fov_;
    pcl::PointCloud<Surfel> pointcloud_;
    std::vector<int> u_mapping_;
    std::vector<int> v_mapping_;
    float initial_confidence_;
    float p_;

protected:
    int computeMappingIndex();
public:
    Map_2_Point(rv::ParameterList parameter_list, float init_confidence);
    ~Map_2_Point();
    pcl::PointCloud<Surfel>& setPointCloud();
    bool generateMappingIndex();
    int getUIndex(int index) {return u_mapping_[index];}
    int getVIndex(int index) {return v_mapping_[index];}
    const pcl::PointCloud<Surfel>& getPointClouds() {return pointcloud_;}
    pcl::PointCloud<Surfel>::Ptr getPointCloudsPtr() {return pointcloud_.makeShared();}
};


#endif //SRC_MAP_2_POINT_H
