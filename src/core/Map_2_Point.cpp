//
// Created by tongda on 2022/1/2.
//

#include "Map_2_Point.h"

const float PI = acos(-1);

Map_2_Point::Map_2_Point(rv::ParameterList parameter_list, float init_confidence) :
        width_(parameter_list["width"]),
        height_(parameter_list["height"]),
        pointcloud_(),
        initial_confidence_(init_confidence)
{
    fov_down_ = parameter_list["fov_down"];
    fov_up_ = parameter_list["fov_up"];
    fov_ = abs(fov_up_) + abs(fov_down_);
    p_ = std::max(width_ / float(360.0), height_ / fov_);
}

pcl::PointCloud<Surfel>& Map_2_Point::setPointCloud() {
    pointcloud_.clear();
    return pointcloud_;
}

bool Map_2_Point::generateMappingIndex() {
    int num_points = pointcloud_.size();
    u_mapping_.resize(num_points);
    v_mapping_.resize(num_points);
    // select points
    for(int i = 0; i < num_points; i++)
    {
        float x = pointcloud_.points[i].x;
        float y = pointcloud_.points[i].y;
        float z = pointcloud_.points[i].z;
        // compute the distance to zero point
        float r_xyz = sqrt(x*x + y*y + z*z);
        // error point
        if(r_xyz == 0)
        {
            pointcloud_.points[i].y = NAN;
            continue;
        }
        // compute u, v coordination
        u_mapping_[i] = (int)(0.5 * width_ * (1 - atan(y / x) / PI));
        v_mapping_[i] = (int)((1 - abs(asin(z / r_xyz) * 180.0 / PI + abs(fov_down_)) / fov_) * height_);
    }

    return false;
}
