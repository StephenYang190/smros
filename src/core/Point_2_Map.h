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
        float point_x = 0.0;
        float radius = 0.0;
        int index = -1;
    };
}

class Point_2_Map {
private:
    int height_, width_;
    float fov_up_, fov_down_, fov_;
    std::shared_ptr<pcl::PointCloud<Surfel>> pointcloud_;
    std::vector<std::vector<frm::mapping_inddex>> maps_;
    float initial_confidence_;
    float p_;

protected:
    int computeMappingIndex();
    bool removeNanPoint();
    void printIndex();
public:
    Point_2_Map(rv::ParameterList parameter_list, float init_confidence);
    ~Point_2_Map();
    std::shared_ptr<pcl::PointCloud<Surfel>> setPointCloud();
    bool generateSurfel(int timestamp);
    std::shared_ptr<pcl::PointCloud<Surfel>> getPointCloudsPtr() {return pointcloud_;}
    const std::vector<std::vector<frm::mapping_inddex>> & getMaps() const {return maps_;};
    int getHeight() {return height_;}
    int getWidth() {return width_;}
    int getPointNum() {return pointcloud_->size();}
    bool clearIndexMap();
    bool generateMappingIndex();
    int getIndex(int u, int v);
};


#endif //SMROS_FRAME_H
