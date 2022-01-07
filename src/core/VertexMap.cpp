//
// Created by tongda on 2021/12/17.
//

#include "VertexMap.h"


const float PI = acos(-1);

VertexMap::VertexMap(rv::ParameterList parameter_list, float init_confidence) :
        width_(parameter_list["width"]),
        height_(parameter_list["height"]),
        initial_confidence_(init_confidence)
{
    fov_down_ = parameter_list["fov_down"];
    fov_up_ = parameter_list["fov_up"];
    fov_ = abs(fov_up_) + abs(fov_down_);
    pointclouds_ = std::make_shared<pcl::PointCloud<Surfel>>();

    maps_.resize(width_);
    for(int i = 0; i < width_; i++)
    {
        maps_[i].resize(height_);
    }
    clearIndexMap();
    p_ = std::max(width_ / float(360.0), height_ / fov_);
}

VertexMap::~VertexMap() {

}

bool VertexMap::setPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_point_clouds) {
    pointclouds_->clear();
    clearIndexMap();
    pcl::copyPointCloud(*input_point_clouds, *pointclouds_);
    return true;
}

bool VertexMap::points2Surfel(int timestamp) {
    computeMappingIndex();

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
            Eigen::Vector3d u1 (pointclouds_->points[u_point_index].x, pointclouds_->points[u_point_index].y, pointclouds_->points[u_point_index].z);
            Eigen::Vector3d v1 (pointclouds_->points[v_point_index].x, pointclouds_->points[v_point_index].y, pointclouds_->points[v_point_index].z);;
            Eigen::Vector3d point_xyz (pointclouds_->points[index].x, pointclouds_->points[index].y, pointclouds_->points[index].z);;
            // compute normal
            Eigen::Vector3d normal = (u1 - point_xyz).cross(v1 - point_xyz);
            normal.normalize();
            // compute radius
            float molecular = sqrt(2.0) * maps_[u][v].radius * p_;
            double dis = -1.0 * point_xyz.dot(normal) / maps_[u][v].radius;
            float denominator = std::min(1.0, std::max(dis, 0.5));
            // set surfel
            pointclouds_->points[index].radius = molecular / denominator;
            pointclouds_->points[index].nx = normal[0];
            pointclouds_->points[index].ny = normal[1];
            pointclouds_->points[index].nz = normal[2];
            pointclouds_->points[index].create_timestamp = timestamp;
            pointclouds_->points[index].update_timestamp = timestamp;
            pointclouds_->points[index].confidence = initial_confidence_;
        }
    }

    // romove nan point
    removeNanPoint();

    return true;
}

bool VertexMap::clearIndexMap() {
    for(int u = 0; u < width_; u++)
    {
        for(int v = 0; v < height_; v++) {
            maps_[u][v].index = -1;
        }
    }
    return false;
}

bool VertexMap::computeMappingIndex() {
    // set index map to -1
    clearIndexMap();
    int num_points = pointclouds_->size();
    // compute the mapping from point to map
    float x, y, z, r_xyz;
    float yaw_angle, pitch_angle;
    int u, v;
    for(int i = 0; i < num_points; i++)
    {
        x = pointclouds_->points[i].x;
        y = pointclouds_->points[i].y;
        z = pointclouds_->points[i].z;
        // compute the distance to zero point
        r_xyz = sqrt(x*x + y*y + z*z);
        // error point
        if(r_xyz < 1e-5)
        {
            pointclouds_->points[i].y = NAN;
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

        bool insert = true;
        // select the nearest point
        if(maps_[u][v].index != -1 && maps_[u][v].radius < r_xyz)
        {
            insert = false;
        }
        if(insert)
        {
            maps_[u][v].radius = r_xyz;
            maps_[u][v].index = i;
            maps_[u][v].point_x = pointclouds_->points[i].x;
        }
    }

    return true;
}

bool VertexMap::removeNanPoint() {
    int num_points = pointclouds_->size();
    // set all point to nan
    for(int i = 0; i < num_points; i++)
    {
        pointclouds_->points[i].x = NAN;
    }
    // recover the point in map
    for(int u = 0; u < width_; u++) {
        for (int v = 0; v < height_; v++) {
            // no point fit in this site
            if (maps_[u][v].index < 0) {
                continue;
            }
            pointclouds_->points[maps_[u][v].index].x = maps_[u][v].point_x;
        }
    }

    // romove nan point
    std::vector<int> mapping;
    pointclouds_->is_dense = false;
    pcl::removeNaNFromPointCloud(*pointclouds_, *pointclouds_, mapping);

    return false;
}

int VertexMap::getIndex(int u, int v) {
    return maps_[u][v].index;
}

void VertexMap::printIndex(){
    int num_point = pointclouds_->size();
    std::cout << "Begin to find irregular point." << std::endl;
    for(int u = 0; u < width_; u++) {
        for (int v = 0; v < height_; v++) {
            // no point fit in this way
            if (maps_[u][v].index < num_point) {
                continue;
            }
            std::cout << u << " " << v << " " << maps_[u][v].radius << " " << std::endl;
        }
    }
    std::cout << "End to find irregular point." << std::endl;
}

bool VertexMap::generateMappingIndex() {
    computeMappingIndex();
    return true;
}
