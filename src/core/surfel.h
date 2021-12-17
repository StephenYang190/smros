//
// Created by tongda on 2021/12/16.
//

#ifndef SMROS_SURFEL_H
#define SMROS_SURFEL_H

#include "pcl_ros/point_cloud.h"

struct Surfel {
    pcl::PointCloud<pcl::PointXYZRGB> points; // points
    float radius; // radius
    float nx, ny, nz; // normal
    float confidence;

    uint32_t timestamp;
    float color, weight, count;
    float r, g, b, w;
};

#endif //SMROS_SURFEL_H
