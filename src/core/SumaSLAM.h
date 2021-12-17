//
// Created by tongda on 2021/12/16.
//

#ifndef SMROS_SUMASLAM_H
#define SMROS_SUMASLAM_H


#include "RangenetAPI.hpp"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


class SumaSLAM {
private:
    std::shared_ptr<RangenetAPI> net_;

public:
    SumaSLAM();
    ~SumaSLAM();
    bool step(const pcl::PointCloud<pcl::PointXYZI> point_clouds_xyzi);
    bool render();
    bool transform(const pcl::PointCloud<pcl::PointXYZI> point_clouds_xyzi,
                   pcl::PointCloud<pcl::PointXYZRGB> &point_clouds_rgb
                   );

};


#endif //SMROS_SUMASLAM_H
