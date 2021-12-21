//
// Created by tongda on 2021/12/19.
//

#include "SurfelMap.h"
#include <pcl/kdtree/kdtree_flann.h>
#include "Frame.h"
#include <cmath>

SurfelMap::SurfelMap(rv::ParameterList parameter_list):
        width_(parameter_list["width"]),
        height_(parameter_list["height"]),
        poses_(),
        surfels_(),
        p_stable_(parameter_list["p_stable"]),
        p_prior_(parameter_list["p_prior"]),
        sigma_angle_2_(parameter_list["sigma_angle"]),
        sigma_distance_2_(parameter_list["sigma_distance"])
{
    odds_p_prior_ = std::log(p_prior_ / (1 - p_prior_));
    initial_confidence_ = std::log(p_stable_ / (1 - p_stable_)) - odds_p_prior_;
    sigma_angle_2_ = sigma_angle_2_ * sigma_angle_2_;
    sigma_distance_2_ = sigma_distance_2_ * sigma_distance_2_;
}

SurfelMap::~SurfelMap() {

}

bool SurfelMap::pushBackPose(Eigen::Matrix4f pose) {
    poses_.push_back(pose);
    return true;
}

Eigen::Matrix4f SurfelMap::getPose(int timestamp) {
    return poses_[timestamp];
}

bool SurfelMap::updateMap(const Frame & current_frame, int timestamp)
{
    // get maps and radius
    std::vector<std::vector<frm::map>> map_ = current_frame.getMaps();
    // create kdtree
    pcl::KdTreeFLANN<Surfel> kdtree;
    kdtree.setInputCloud(surfels_.makeShared());

    std::vector<int> pointIdxKNNSearch(1);
    std::vector<float> pointKNNSquaredDistance(1);
    for(int u = 0; u < width_; u++)
    {
        for(int v = 0; v < height_; v++)
        {
            // find nearest point
            Surfel search_point;
            search_point.x = map_[u][v].vertex_map[0];
            search_point.y = map_[u][v].vertex_map[1];
            search_point.z = map_[u][v].vertex_map[2];
            kdtree.nearestKSearch(search_point, 1, pointIdxKNNSearch, pointKNNSquaredDistance);
            // get nearest point data
            int index = pointIdxKNNSearch[0];
            float distance = pointKNNSquaredDistance[0];
            Eigen::Vector3d vsp({surfels_[index].x, surfels_[index].y, surfels_[index].z});
            Eigen::Vector3d nsp({surfels_[index].nx, surfels_[index].ny, surfels_[index].nz});
            float rsp = surfels_[index].radius;
            // compute reference
            float metric1 = std::abs(nsp.dot(map_[u][v].vertex_map - vsp));
            Eigen::Vector3d normal_ns_nsp = map_[u][v].normal_map.cross(nsp);
            float metric2 = std::sqrt(normal_ns_nsp.dot(normal_ns_nsp));

        }
    }
}

bool SurfelMap::mapInitial(const Frame & current_frame)
{
    // push init pose
    poses_.push_back(Eigen::Matrix4f::Identity());
    // get maps and radius
    std::vector<std::vector<frm::map>> map_ = current_frame.getMaps();
    // create map
    for(int u = 0; u < width_; u++)
    {
        for(int v = 0; v < height_; v++)
        {
            Surfel surfel;
            surfel.x = map_[u][v].vertex_map[0];
            surfel.y = map_[u][v].vertex_map[1];
            surfel.z = map_[u][v].vertex_map[2];

            surfel.radius = map_[u][v].radius;

            surfel.nx = map_[u][v].normal_map[0];
            surfel.ny = map_[u][v].normal_map[1];
            surfel.nz = map_[u][v].normal_map[2];

            surfel.create_timestamp = 0;
            surfel.update_timestamp = 0;

            surfel.confidence = initial_confidence_;
            surfels_.push_back(surfel);
        }
    }
}

float SurfelMap::updateConfidence(float confidence, float angle_2, float distance_2) {
    float angle_error = std::exp(-1.0 * angle_2 / sigma_angle_2_);
    float distance_error = std::exp(-1.0 * distance_2 / sigma_angle_2_);
    float odds_error = std::log(p_stable_ * angle_error * distance_error);
    return confidence + odds_error - odds_p_prior_;
}
