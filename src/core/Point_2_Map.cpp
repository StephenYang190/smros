//
// Created by tongda on 2021/12/17.
//

#include "Point_2_Map.h"


const float PI = acos(-1);

Point_2_Map::Point_2_Map(rv::ParameterList parameter_list, float init_confidence) :
        width_(parameter_list["width"]),
        height_(parameter_list["height"]),
        initial_confidence_(init_confidence)
{
    fov_down_ = parameter_list["fov_down"];
    fov_up_ = parameter_list["fov_up"];
    fov_ = abs(fov_up_) + abs(fov_down_);
    pointcloud_ = std::make_shared<pcl::PointCloud<Surfel>>();

    maps_.resize(width_);
    for(int i = 0; i < width_; i++)
    {
        maps_[i].resize(height_);
    }
    clearIndexMap();
    p_ = std::max(width_ / float(360.0), height_ / fov_);
}

Point_2_Map::~Point_2_Map() {

}

std::shared_ptr<pcl::PointCloud<Surfel>> Point_2_Map::setPointCloud() {
    pointcloud_->clear();
    clearIndexMap();
    return pointcloud_;
}

bool Point_2_Map::generateSurfel(int timestamp) {
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
            Eigen::Vector3d u1 (pointcloud_->points[u_point_index].x, pointcloud_->points[u_point_index].y, pointcloud_->points[u_point_index].z);
            Eigen::Vector3d v1 (pointcloud_->points[v_point_index].x, pointcloud_->points[v_point_index].y, pointcloud_->points[v_point_index].z);;
            Eigen::Vector3d point_xyz (pointcloud_->points[index].x, pointcloud_->points[index].y, pointcloud_->points[index].z);;
            // compute normal
            Eigen::Vector3d normal = (u1 - point_xyz).cross(v1 - point_xyz);
            normal.normalize();
            // compute radius
            float molecular = sqrt(2.0) * maps_[u][v].radius * p_;
            double dis = -1.0 * point_xyz.dot(normal) / maps_[u][v].radius;
            float denominator = std::min(1.0, std::max(dis, 0.5));
            // set surfel
            pointcloud_->points[index].radius = molecular / denominator;
            pointcloud_->points[index].nx = normal[0];
            pointcloud_->points[index].ny = normal[1];
            pointcloud_->points[index].nz = normal[2];
            pointcloud_->points[index].create_timestamp = timestamp;
            pointcloud_->points[index].update_timestamp = timestamp;
            pointcloud_->points[index].confidence = initial_confidence_;
        }
    }

    // romove nan point
    removeNanPoint();

    return true;
}

bool Point_2_Map::clearIndexMap() {
    for(int u = 0; u < width_; u++)
    {
        for(int v = 0; v < height_; v++) {
            maps_[u][v].index = -1;
        }
    }
    return false;
}

int Point_2_Map::computeMappingIndex() {
    clearIndexMap();
    int num_points = pointcloud_->size();
    // select points
    for(int i = 0; i < num_points; i++)
    {
        float x = pointcloud_->points[i].x;
        float y = pointcloud_->points[i].y;
        float z = pointcloud_->points[i].z;
        // compute the distance to zero point
        float r_xyz = sqrt(x*x + y*y + z*z);
        // error point
        if(r_xyz < 1e-5)
        {
            pointcloud_->points[i].y = NAN;
            continue;
        }
        // compute u, v coordination
        int u = (int)(0.5 * width_ * (1 - atan(y / x) / PI));
        int v = (int)((1 - abs(asin(z / r_xyz) * 180.0 / PI + abs(fov_down_)) / fov_) * height_);
        if(u >= width_ || v >= height_ || u < 0 || v < 0){
            continue;
        }

        bool insert = true;
        if(maps_[u][v].index != -1 && maps_[u][v].radius < r_xyz)
        {
            insert = false;
        }
        if(insert)
        {
            maps_[u][v].radius = r_xyz;
            maps_[u][v].index = i;
            maps_[u][v].point_x = pointcloud_->points[i].x;
        }
    }

    return num_points;
}

bool Point_2_Map::removeNanPoint() {
    int num_points = pointcloud_->size();
    // select points
    for(int i = 0; i < num_points; i++)
    {
        pointcloud_->points[i].x = NAN;
    }
    for(int u = 0; u < width_; u++) {
        for (int v = 0; v < height_; v++) {
            // no point fit in this way
            if (maps_[u][v].index < 0) {
                continue;
            }
            pointcloud_->points[maps_[u][v].index].x = maps_[u][v].point_x;
        }
    }

    // romove nan point
    std::vector<int> mapping;
    pointcloud_->is_dense = false;
    pcl::removeNaNFromPointCloud(*pointcloud_, *pointcloud_, mapping);

    return false;
}

bool Point_2_Map::generateMappingIndex() {
    computeMappingIndex();
    return false;
}

int Point_2_Map::getIndex(int u, int v) {
    return maps_[u][v].index;
}

void Point_2_Map::printIndex(){
    int num_point = pointcloud_->size();
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
