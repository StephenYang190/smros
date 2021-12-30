//
// Created by tongda on 2021/12/19.
//

#include "SurfelMap.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/impl/filter.hpp>


SurfelMap::SurfelMap(rv::ParameterList parameter_list):
        width_(parameter_list["width"]),
        height_(parameter_list["height"]),
        poses_(),
        surfels_(0),
        active_map_(),
        p_stable_(parameter_list["p_stable"]),
        p_prior_(parameter_list["p_prior"]),
        sigma_angle_2_(parameter_list["sigma_angle"]),
        sigma_distance_2_(parameter_list["sigma_distance"]),
        distance_thred_(parameter_list["distance_thred"]),
        angle_thred_(parameter_list["angle_thred"]),
        max_distance_(parameter_list["max_distance"]),
        gamma_(parameter_list["gamma"]),
        confidence_thred_(parameter_list["confidence_threshold"]),
        time_gap_(parameter_list["time_gap"]),
        init_pose_(Eigen::Matrix4f::Identity()),
        my_global_map_()
{
    odds_p_prior_ = std::log(p_prior_ / (1 - p_prior_));
    initial_confidence_ = std::log(p_stable_ / (1 - p_stable_)) - odds_p_prior_;
    sigma_angle_2_ = sigma_angle_2_ * sigma_angle_2_;
    sigma_distance_2_ = sigma_distance_2_ * sigma_distance_2_;
    angle_thred_ = std::sin(angle_thred_);
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

bool SurfelMap::mapInitial(const pcl::PointCloud<Surfel> & pointcloud)
{
    std::clock_t start_time = std::clock();
    // push init pose
    poses_.push_back(Eigen::Matrix4f::Identity());

    // create map
    surfels_.push_back(pointcloud);
    std::clock_t end_time = std::clock();
    std::cout << "Initial map in surfel map. " << "using " << (float)(end_time - start_time) / CLOCKS_PER_SEC << std::endl;
    return true;
}

float SurfelMap::updateConfidence(float confidence, float angle_2, float distance_2) {
    float angle_error = std::exp(-1.0 * angle_2 / sigma_angle_2_);
    float distance_error = std::exp(-1.0 * distance_2 / sigma_angle_2_);
    float odds_error = p_stable_ * angle_error * distance_error;
    float odds_p_stable = std::log(odds_error / (1 - odds_error));
    return confidence + odds_p_stable - odds_p_prior_;
}

bool SurfelMap::initialSurfel(frm::map new_surfel, int timestamp, Surfel & surfel)
{
    surfel.x = new_surfel.vertex_map[0];
    surfel.y = new_surfel.vertex_map[1];
    surfel.z = new_surfel.vertex_map[2];

    surfel.radius = new_surfel.radius;

    surfel.nx = new_surfel.normal_map[0];
    surfel.ny = new_surfel.normal_map[1];
    surfel.nz = new_surfel.normal_map[2];

    surfel.create_timestamp = timestamp;
    surfel.update_timestamp = timestamp;

    surfel.confidence = initial_confidence_;

    return true;
}

bool SurfelMap::initialSurfel(int timestamp, Surfel & surfel)
{
    surfel.create_timestamp = timestamp;
    surfel.update_timestamp = timestamp;

    surfel.confidence = initial_confidence_;

    return true;
}

bool SurfelMap::updateActiveMap(int timestamp)
{
    std::clock_t start_time = std::clock();

    for(int i = 0; i < active_map_.size(); i++)
    {
        int cr_time = active_map_[i].create_timestamp;
        int up_time = active_map_[i].update_timestamp;
//        if(active_map_[i].confidence < confidence_thred_)
//        {
//            active_map_[i].x = NAN;
//        }

//        std::cout << active_map_[i].update_timestamp << " < " << timestamp - time_gap_ << std::endl;
        if(cr_time < (timestamp - time_gap_))
        {
            if(cr_time >= surfels_.size())
            {
                while (surfels_.size() <= cr_time)
                {
                    pcl::PointCloud<Surfel> new_time_clouds;
                    surfels_.push_back(new_time_clouds);
                }
            }
            surfels_[cr_time].push_back(active_map_[i]);
            active_map_[i].x = NAN;
        }
    }

    std::vector<int> mapping;
    active_map_.is_dense = false;
    pcl::removeNaNFromPointCloud(active_map_, active_map_, mapping);

    std::clock_t end_time = std::clock();
    std::cout << "Updating active map in surfel map. " << "using " << (float)(end_time - start_time) / CLOCKS_PER_SEC << std::endl;
    return true;
}

bool SurfelMap::generateMap(pcl::PointCloud<Surfel> & global_map)
{
    if (surfels_.empty())
    {
        return false;
    }
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    for(int i = 0; i < surfels_.size(); i++)
    {
        pose *= poses_[i];
        pcl::PointCloud<Surfel> transform_result;
        pcl::transformPointCloud(surfels_[i], transform_result, pose);
        global_map += transform_result;
        if(i == surfels_.size() - 1)
        {
            my_global_map_ += transform_result;
        }
    }

    return true;
}

//bool
//SurfelMap::updateMap(const pcl::PointCloud<Surfel> &align_point,
//                     const pcl::CorrespondencesPtr mapping,
//                     int timestamp,
//                     std::shared_ptr<Frame> current_frame) {
//    std::clock_t start_time = std::clock();
//
//    pcl::PointCloud<Surfel>::Ptr pointcloud = current_frame->getPointCloudsPtr();
//    int num_point = pointcloud->size();
//    std::vector<bool> updated_point(num_point, false);
//
//    for(int i = 0; i < mapping->size(); i++)
//    {
//        int src_index = (*mapping)[i].index_query;
//        int tgt_index = (*mapping)[i].index_match;
//
//        updated_point[src_index] = true;
//        // get nearest point data
//        Eigen::Vector3d vsp({active_map_[tgt_index].x, active_map_[tgt_index].y, active_map_[tgt_index].z});
//        Eigen::Vector3d nsp({active_map_[tgt_index].nx, active_map_[tgt_index].ny, active_map_[tgt_index].nz});
//        float rsp = active_map_[tgt_index].radius;
//
//        Eigen::Vector3d vs({align_point[src_index].x, align_point[src_index].y, align_point[src_index].z});
//        Eigen::Vector3d ns({align_point[src_index].nx, align_point[src_index].ny, align_point[src_index].nz});
//        float rs = align_point[src_index].radius;
//        // compute reference
//        float metric1 = std::abs(nsp.dot(vs - vsp));
//        Eigen::Vector3d normal_ns_nsp = ns.cross(nsp);
//        float metric2 = std::sqrt(normal_ns_nsp.dot(normal_ns_nsp));
//        // verify threshold
//        if(metric1 < distance_thred_ && metric2 < angle_thred_)
//        {
//            active_map_[tgt_index].confidence = updateConfidence(active_map_[tgt_index].confidence,
//                                                          metric2*metric2, metric1*metric1);
//            if(rsp > rs)
//            {
//                active_map_[tgt_index].x = (1 - gamma_) * align_point[src_index].x + gamma_ * active_map_[tgt_index].x;
//                active_map_[tgt_index].y = (1 - gamma_) * align_point[src_index].y + gamma_ * active_map_[tgt_index].y;
//                active_map_[tgt_index].z = (1 - gamma_) * align_point[src_index].z + gamma_ * active_map_[tgt_index].z;
//
//                active_map_[tgt_index].nx = (1 - gamma_) * align_point[src_index].nx + gamma_ * active_map_[tgt_index].nx;
//                active_map_[tgt_index].ny = (1 - gamma_) * align_point[src_index].ny + gamma_ * active_map_[tgt_index].ny;
//                active_map_[tgt_index].nz = (1 - gamma_) * align_point[src_index].nz + gamma_ * active_map_[tgt_index].nz;
//
//                active_map_[tgt_index].radius = align_point[src_index].radius;
//
//                active_map_[tgt_index].update_timestamp = timestamp;
//            }
//        }
//        else
//        {
//            active_map_[tgt_index].confidence = updateConfidence(active_map_[tgt_index].confidence,
//                                                          metric2*metric2, metric1*metric1);
//            active_map_[tgt_index].update_timestamp = timestamp;
//            active_map_.push_back(pointcloud->points[src_index]);
//        }
//    }
//
//    for(int i = 0; i < num_point; i++)
//    {
//        if(updated_point[i])
//        {
//            continue;
//        }
//        active_map_.push_back(pointcloud->points[i]);
//    }
//
//    std::clock_t end_time = std::clock();
//    std::cout << "Updating map in surfel map. " << "using " << (float)(end_time - start_time) / CLOCKS_PER_SEC << std::endl;
//
//    updateActiveMap(timestamp);
//    return true;
//}

bool SurfelMap::updateMap(const pcl::CorrespondencesPtr mapping,
                     int timestamp,
                     std::shared_ptr<Frame> current_frame) {
//    std::clock_t start_time = std::clock();
    // create a new timestamp surfel
    pcl::PointCloud<Surfel> new_time_clouds;
    surfels_.push_back(new_time_clouds);
    // transofor active map to input local coordination
    pcl::PointCloud<Surfel> transform_result;
    pcl::transformPointCloud(active_map_, transform_result, poses_[timestamp].inverse());
    // get input point
    pcl::PointCloud<Surfel>::Ptr pointcloud = current_frame->getPointCloudsPtr();
    int num_point = pointcloud->size();
    std::vector<bool> updated_point(num_point, false);

    for(int i = 0; i < mapping->size(); i++)
    {
        int src_index = (*mapping)[i].index_query;
        int tgt_index = (*mapping)[i].index_match;

        updated_point[src_index] = true;
        // get nearest point data
        Eigen::Vector3d vsp({transform_result[tgt_index].x, transform_result[tgt_index].y, transform_result[tgt_index].z});
        Eigen::Vector3d nsp({transform_result[tgt_index].nx, transform_result[tgt_index].ny, transform_result[tgt_index].nz});
        float rsp = transform_result[tgt_index].radius;

        Eigen::Vector3d vs({pointcloud->points[src_index].x, pointcloud->points[src_index].y, pointcloud->points[src_index].z});
        Eigen::Vector3d ns({pointcloud->points[src_index].nx, pointcloud->points[src_index].ny, pointcloud->points[src_index].nz});
        std::cout << "vs:" << vs << std::endl;
        std::cout << "vsp:" << vsp << std::endl;
        float rs = pointcloud->points[src_index].radius;
        // compute reference
        float metric1 = std::abs(nsp.dot(vs - vsp));
        Eigen::Vector3d normal_ns_nsp = ns.cross(nsp);
        float metric2 = std::sqrt(normal_ns_nsp.dot(normal_ns_nsp));
        // verify threshold
        if(metric1 < distance_thred_ && metric2 < angle_thred_ && rsp < rs)
        {

//            pointcloud->points[src_index].x = (1 - gamma_) * pointcloud->points[src_index].x + gamma_ * transform_result[tgt_index].x;
//            pointcloud->points[src_index].y = (1 - gamma_) * pointcloud->points[src_index].y + gamma_ * transform_result[tgt_index].y;
//            pointcloud->points[src_index].z = (1 - gamma_) * pointcloud->points[src_index].z + gamma_ * transform_result[tgt_index].z;
//
//            pointcloud->points[src_index].nx = (1 - gamma_) * pointcloud->points[src_index].nx + gamma_ * transform_result[tgt_index].nx;
//            pointcloud->points[src_index].ny = (1 - gamma_) * pointcloud->points[src_index].ny + gamma_ * transform_result[tgt_index].ny;
//            pointcloud->points[src_index].nz = (1 - gamma_) * pointcloud->points[src_index].nz + gamma_ * transform_result[tgt_index].nz;
//
//            pointcloud->points[src_index].radius = pointcloud->points[src_index].radius;
        }

        else
        {
//            surfels_[timestamp].push_back(pointcloud->points[src_index]);
        }

    }

    for(int i = 0; i < num_point; i++)
    {
        if(updated_point[i])
        {
            continue;
        }
        surfels_[timestamp].push_back(pointcloud->points[i]);
    }

//    std::clock_t end_time = std::clock();
//    std::cout << "Updating map in surfel map. " << "using " << (float)(end_time - start_time) / CLOCKS_PER_SEC << std::endl;
    return true;
}

bool SurfelMap::generateActiveMap(int timestamp)
{
    active_map_.clear();
    if (surfels_.size() == 0)
    {
        return false;
    }
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    // generate map
    active_map_ += surfels_[timestamp - 1];
    for(int i = timestamp - 1; i > timestamp - time_gap_ && i > 0; i--)
    {
        pose *= poses_[i].inverse();
        pcl::PointCloud<Surfel> transform_result;
        pcl::transformPointCloud(surfels_[i - 1], transform_result, pose);
        active_map_ += transform_result;
    }
//    int i = std::max(0, timestamp - time_gap_);
//    for(; i < timestamp; i++)
//    {
//        pose *= poses_[i].inverse();
//        pcl::PointCloud<Surfel> transform_result;
//        pcl::transformPointCloud(surfels_[i], transform_result, pose);
//        active_map_ += transform_result;
//    }

    return true;
}