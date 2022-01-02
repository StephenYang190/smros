//
// Created by tongda on 2021/12/17.
//

#ifndef SMROS_FRAME_H
#define SMROS_FRAME_H


#include <pcl_conversions/pcl_conversions.h>
#include <rv/ParameterList.h>
#include <memory>
#include <Eigen/Dense>
#include "surfel.h"
#include <pcl/filters/filter.h>
#include <cmath>
#include <pcl/filters/impl/filter.hpp>


namespace frm{
    struct mapping_inddex{
        float point_x;
        int index;
        float radius;
    };
}

class Point_2_Map {
private:
    int height_, width_;
    float fov_up_, fov_down_, fov_;
    pcl::PointCloud<Surfel> pointcloud_;
    std::vector<std::vector<frm::mapping_inddex>> maps_;
    float initial_confidence_;
    float p_;

protected:
    int computeMappingIndex();
    bool removeNanPoint();
public:
    Point_2_Map(rv::ParameterList parameter_list, float init_confidence);
    ~Point_2_Map();
    pcl::PointCloud<Surfel>& setPointCloud();
    bool generateSurfel(int timestamp);
    const pcl::PointCloud<Surfel>& getPointClouds() {return pointcloud_;}
    pcl::PointCloud<Surfel>::Ptr getPointCloudsPtr() {return pointcloud_.makeShared();}
    const std::vector<std::vector<frm::mapping_inddex>> & getMaps() const {return maps_;};
    int getHeight() {return height_;}
    int getWidth() {return width_;}
    bool clearIndexMap();
    bool generateMappingIndex();
    int getIndex(int u, int v);
};


#endif //SMROS_FRAME_H
