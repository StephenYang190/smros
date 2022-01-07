/* VertexMap class
 * Create by Tongda Yang
 * This class is used to store and compute the radius, normal and the index on the vertex map
 * We only store the points which on the vertex map
 * */

#ifndef SMROS_FRAME_H
#define SMROS_FRAME_H

#include <pcl_conversions/pcl_conversions.h>
#include <rv/ParameterList.h>
#include <memory>
#include <cmath>
#include <pcl/filters/impl/filter.hpp>

#include "surfel.h"

namespace frm{
    struct mapping_index{
        float point_x = 0.0;
        float radius = 0.0;
        int index = -1;
    };
}

class VertexMap {
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
    // pixel size
    float p_;

protected:
    // compute the matching between the vertex map and point clouds
    bool computeMappingIndex();
    // remove Nan point and point not in the map from point clouds
    bool removeNanPoint();
    // print index which is out of range
    void printIndex();
    // set index map to -1
    bool clearIndexMap();

public:
    VertexMap(rv::ParameterList parameter_list, float init_confidence);
    ~VertexMap();
    // set the point clouds xyz
    bool setPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_point_clouds);
    // transform point clouds to surfel based point clouds(compute the radius and other parameters)
    bool points2Surfel(int timestamp);
    // get point clouds ptr
    std::shared_ptr<pcl::PointCloud<Surfel>> getPointCloudsPtr() {return pointclouds_;}
    // get point number
    int getPointNum() {return pointclouds_->size();}
    // get point index on vertex map
    int getIndex(int u, int v);
    // generate mapping index
    bool generateMappingIndex();
};


#endif //SMROS_FRAME_H
