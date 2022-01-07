//
// Created by tongda on 2022/1/2.
//

#include "PointIndex.h"

const float PI = acos(-1);

PointIndex::PointIndex(rv::ParameterList parameter_list) :
        width_(parameter_list["width"]),
        height_(parameter_list["height"]),
        pointclouds_()
{
    fov_down_ = parameter_list["fov_down"];
    fov_up_ = parameter_list["fov_up"];
    fov_ = abs(fov_up_) + abs(fov_down_);
}

pcl::PointCloud<Surfel>& PointIndex::setPointCloud() {
    pointclouds_.clear();
    return pointclouds_;
}

bool PointIndex::generateMappingIndex() {
    int num_points = pointclouds_.size();
    u_list.resize(num_points);
    v_list_.resize(num_points);
    // compute index
    float x, y, z, r_xyz;
    float yaw_angle, pitch_angle;
    int u, v;
    for(int i = 0; i < num_points; i++)
    {
        x = pointclouds_.points[i].x;
        y = pointclouds_.points[i].y;
        z = pointclouds_.points[i].z;
        // compute the distance to zero point
        r_xyz = sqrt(x*x + y*y + z*z);
        // error point
        if(r_xyz < 1e-5)
        {
            pointclouds_.points[i].y = NAN;
            continue;
        }
        // compute the yaw angle (max:360)
        yaw_angle = atan(y / x) * 180.0 / PI;
        // compute u coordination
        u = (int)(0.5 * width_ * (1 - yaw_angle / 360));
        // compute the pitch angle (max:360)
        pitch_angle = asin(z / r_xyz) * 180.0 / PI;
        // compute v coordination
        v = (int)((1 - abs(pitch_angle + abs(fov_down_)) / fov_) * height_);
        // coordination can not out of range
        u = std::max(std::min(u, width_ - 1), 0);
        v = std::max(std::min(v, height_ - 1), 0);
        // compute u, v coordination
        u_list[i] = u;
        v_list_[i] = v;
    }

    return false;
}
