//
// Created by tongda on 2021/12/17.
//

#include "Frame.h"
#include <cmath>

const float PI = acos(-1);

Frame::Frame(rv::ParameterList parameter_list) :
        width_(parameter_list["width"]),
        height_(parameter_list["height"]),
        pointcloud_(),
        labels_(),
        label_probability_()
{
    fov_down_ = parameter_list["fov_down"];
    fov_up_ = parameter_list["fov_up"];
    fov_ = fov_up_ + fov_down_;

    maps_.resize(width_);
    for(int i = 0; i < width_; i++)
    {
        maps_[i].resize(height_);
    }
}

Frame::~Frame() {

}

pcl::PointCloud<pcl::PointXYZRGB>& Frame::setPointCloud() {
    pointcloud_.clear();
    return pointcloud_;
}

bool Frame::generateMap() {
    int num_points = pointcloud_.size();
    // transform pointcloud to vertex map and semantic map
    for(int i = 0; i < num_points; i++)
    {
        float x = pointcloud_.points[i].x;
        float y = pointcloud_.points[i].y;
        float z = pointcloud_.points[i].z;

        float r_xyz = sqrt(x*x + y*y + z*z);

        int u = (int)(0.5 * width_ * (1 - atan(y / x) / PI));
        int v = (int)((1 - (asin(z / r_xyz) + fov_up_) / fov_) * height_);

        maps_[u][v].vertex_map[0] = x;
        maps_[u][v].vertex_map[1] = y;
        maps_[u][v].vertex_map[2] = z;

        maps_[u][v].semantic_map = labels_[i];
    }
    // compute normal
    for(int u = 0; u < width_; u++)
    {
        for(int v = 0; v < height_; v++)
        {
            Eigen::Vector3d u1 = (u + 1 < width_ ? maps_[u + 1][v].vertex_map : maps_[0][v].vertex_map);
            Eigen::Vector3d v1 = (v + 1 < height_ ? maps_[u][v + 1].vertex_map : maps_[u][0].vertex_map);

            maps_[u][v].normal_map = (u1 - maps_[u][v].vertex_map).cross(v1 - maps_[u][v].vertex_map);
        }
    }
    return true;
}
