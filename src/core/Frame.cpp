//
// Created by tongda on 2021/12/17.
//

#include "Frame.h"


const float PI = acos(-1);

Frame::Frame(rv::ParameterList parameter_list, float init_confidence) :
        width_(parameter_list["width"]),
        height_(parameter_list["height"]),
        pointcloud_(),
        labels_(),
        label_probability_(),
        initial_confidence_(init_confidence)
{
    fov_down_ = parameter_list["fov_down"];
    fov_up_ = parameter_list["fov_up"];
    fov_ = abs(fov_up_) + abs(fov_down_);

    maps_.resize(width_);
    for(int i = 0; i < width_; i++)
    {
        maps_[i].resize(height_);
    }
    clearIndexMap();
    p_ = std::max(width_ / float(360.0), height_ / fov_);
}

Frame::~Frame() {

}

pcl::PointCloud<Surfel>& Frame::setPointCloud() {
    pointcloud_.clear();
    clearIndexMap();
    return pointcloud_;
}

bool Frame::generateSurfel(int timestamp) {
    int num_points = pointcloud_.size();
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
        int u = (int)(0.5 * width_ * (1 - atan(y / x) / PI));
        int v = (int)((1 - abs(asin(z / r_xyz) * 180.0 / PI + abs(fov_down_)) / fov_) * height_);

        bool insert = true;
        if(maps_[u][v].index != -1 && maps_[u][v].radius < r_xyz)
        {
            insert = false;
        }
        if(insert)
        {
            maps_[u][v].radius = r_xyz;
            maps_[u][v].index = i;
            maps_[u][v].vertex_map = pointcloud_.points[i].x;
        }
        pointcloud_.points[i].x = NAN;
    }
    // transform pointcloud to vertex map and semantic map
//    for(int i = 0; i < num_points; i++)
//    {
//        float x = pointcloud_.points[i].x;
//        float y = pointcloud_.points[i].y;
//        float z = pointcloud_.points[i].z;
//
//        float r_xyz = sqrt(x*x + y*y + z*z);
//
//        if(r_xyz == 0)
//        {
//            pointcloud_.points[i].y = NAN;
//            continue;
//        }
//
//        int u = (int)(0.5 * width_ * (1 - atan(y / x) / PI));
//        int v = (int)((1 - abs(asin(z / r_xyz) * 180.0 / PI + abs(fov_down_)) / fov_) * height_);
//
//
//        if(u > width_ || v > height_)
//        {
//            std::cout << i << std::endl;
//            std::cout << u << ", " << v << std::endl;
//            std::cout << abs(asin(z / r_xyz) * 180.0 / PI + abs(fov_down_)) << std::endl;
//            std::cout << fov_ << std::endl;
//        }
//
//        maps_[u][v].vertex_map[0] = x;
//        maps_[u][v].vertex_map[1] = y;
//        maps_[u][v].vertex_map[2] = z;
//
//        maps_[u][v].semantic_map = labels_[i];
//
//        maps_[u][v].radius = r_xyz * r_xyz;
//        maps_[u][v].index = i;
//        pointcloud_.points[i].x = NAN;
//    }
    std::vector<int> stay_point_index;
    // compute normal
    for(int u = 0; u < width_; u++)
    {
        for(int v = 0; v < height_; v++)
        {
            // no point fit in this way
            if(maps_[u][v].index < 0)
            {
                continue;
            }
            // save the index
            int index = maps_[u][v].index;
            stay_point_index.push_back(index);
            // find nearest point
            int u_index = (u + 1) % width_, v_index = (v + 1) % height_;
            while(true)
            {
                if (maps_[u_index][v].index != -1)
                    break;
                u_index = (u_index + 1) % width_;
            }
            while(true)
            {
                if (maps_[u][v_index].index != -1)
                    break;
                v_index = (v_index + 1) % height_;
            }
            // get the point
            int u_point_index = maps_[u_index][v].index;
            int v_point_index = maps_[u][v_index].index;
            Eigen::Vector3d u1 (pointcloud_[u_point_index].x, pointcloud_[u_point_index].y, pointcloud_[u_point_index].z);
            Eigen::Vector3d v1 (pointcloud_[v_point_index].x, pointcloud_[v_point_index].y, pointcloud_[v_point_index].z);;
            Eigen::Vector3d point_xyz (pointcloud_[index].x, pointcloud_[index].y, pointcloud_[index].z);;
            // compute normal
            maps_[u][v].normal_map = (u1 - point_xyz).cross(v1 - point_xyz);
            maps_[u][v].normal_map.normalize();
            // compute radius
            float molecular = sqrt(2.0) * maps_[u][v].radius * p_;
            double dis = -1.0 * point_xyz.dot(maps_[u][v].normal_map) / maps_[u][v].radius;
            float denominator = std::min(1.0, std::max(dis, 0.5));
            // set surfel
            pointcloud_.points[index].x = maps_[u][v].vertex_map;
            pointcloud_.points[index].radius = molecular / denominator;
            pointcloud_.points[index].nx = maps_[u][v].normal_map[0];
            pointcloud_.points[index].ny = maps_[u][v].normal_map[1];
            pointcloud_.points[index].nz = maps_[u][v].normal_map[2];
            pointcloud_.points[index].create_timestamp = timestamp;
            pointcloud_.points[index].update_timestamp = timestamp;
            pointcloud_.points[index].confidence = initial_confidence_;
        }
    }

    // romove nan point
    std::vector<int> mapping;
    pointcloud_.is_dense = false;
    pcl::removeNaNFromPointCloud(pointcloud_, pointcloud_, mapping);

    return true;
}

bool Frame::clearIndexMap() {
    for(int u = 0; u < width_; u++)
    {
        for(int v = 0; v < height_; v++) {
            maps_[u][v].index = -1;
        }
    }
    return false;
}
