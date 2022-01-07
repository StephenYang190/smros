/* PointIndex class
 * Create by Tongda Yang
 * This class is used to compute and store the index in vertex map for each points
 * */

#ifndef SRC_POINTINDEX_H
#define SRC_POINTINDEX_H

#include <rv/ParameterList.h>
#include <memory>
#include "surfel.h"
#include <cmath>

class PointIndex {
private:
    // height and width of vertex map
    int height_, width_;
    // fov up and down for the range of interesting(ROI)
    float fov_up_, fov_down_, fov_;
    // point clouds
    pcl::PointCloud<Surfel> pointclouds_;
    // u index list
    std::vector<int> u_list;
    // v index list
    std::vector<int> v_list_;

protected:

public:
    PointIndex(rv::ParameterList parameter_list);
    ~PointIndex() {};
    // get reference of point clouds
    pcl::PointCloud<Surfel>& setPointCloud();
    // find index of each point
    bool generateMappingIndex();
    // get u index
    int getUIndex(int index) {return u_list[index];}
    // get v index
    int getVIndex(int index) {return v_list_[index];}
    // get point clouds ptr
    pcl::PointCloud<Surfel>::Ptr getPointCloudsPtr() {return pointclouds_.makeShared();}
};


#endif //SRC_POINTINDEX_H
