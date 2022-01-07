//
// Created by tongda on 2021/12/17.
//

#ifndef SMROS_FRAME_H
#define SMROS_FRAME_H
/* VertexMap class
 * Create by Tongda Yang
 * This class is used to store and compute the radius, normal and the index on the vertex map
 * */

#include <pcl_conversions/pcl_conversions.h>
#include <rv/ParameterList.h>
#include <memory>
#include <Eigen/Dense>
#include "surfel.h"
#include <pcl/filters/filter.h>
#include <cmath>
#include <pcl/filters/impl/filter.hpp>


namespace frm{
    struct mapping_index{
        float point_x = 0.0;
        float radius = 0.0;
        int index = -1;
    };
}

class VertexMap {
// private value
private:
    // height and width of vertex map
    int height_, width_;
    // fov up and down for the range of interesting(ROI)
    float fov_up_, fov_down_, fov_;
    // point clouds
    std::shared_ptr<pcl::PointCloud<Surfel>> pointclouds_;
    // vertex map, which store the index of point in point clouds
    std::vector<std::vector<frm::mapping_index>> maps_;
    // initial confidence
    float initial_confidence_;
    //
    float p_;
// protected function
protected:
    // coompute the matching between the vertex map and point clouds
    int computeMappingIndex();
    bool removeNanPoint();
    void printIndex();
// public function
public:
    VertexMap(rv::ParameterList parameter_list, float init_confidence);
    ~VertexMap();
    std::shared_ptr<pcl::PointCloud<Surfel>> setPointCloud();
    bool generateSurfel(int timestamp);
    std::shared_ptr<pcl::PointCloud<Surfel>> getPointCloudsPtr() {return pointclouds_;}
    const std::vector<std::vector<frm::mapping_index>> & getMaps() const {return maps_;};
    int getHeight() {return height_;}
    int getWidth() {return width_;}
    int getPointNum() {return pointclouds_->size();}
    bool clearIndexMap();
    bool generateMappingIndex();
    int getIndex(int u, int v);
};


#endif //SMROS_FRAME_H
