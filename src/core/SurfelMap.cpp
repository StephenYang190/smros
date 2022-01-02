//
// Created by tongda on 2021/12/19.
//

#include "SurfelMap.h"



SurfelMap::SurfelMap(rv::ParameterList parameter_list):
        width_(parameter_list["width"]),
        height_(parameter_list["height"]),
        poses_(),
        surfels_(0),
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
        active_map_index_in_map_(time_gap_, -1)
{
    odds_p_prior_ = std::log(p_prior_ / (1 - p_prior_));
    initial_confidence_ = std::log(p_stable_ / (1 - p_stable_)) - odds_p_prior_;
    sigma_angle_2_ = sigma_angle_2_ * sigma_angle_2_;
    sigma_distance_2_ = sigma_distance_2_ * sigma_distance_2_;
    angle_thred_ = std::sin(angle_thred_);
    active_map_ = std::make_shared<Point_2_Map>(parameter_list, initial_confidence_);
    param_ = parameter_list;
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

bool SurfelMap::mapInitial(const pcl::PointCloud<Surfel> & pointcloud, Eigen::Matrix4f init_pose)
{
    std::clock_t start_time = std::clock();
    // push init pose
    poses_.push_back(init_pose);

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

bool SurfelMap::generateMap(pcl::PointCloud<Surfel> & global_map)
{
    if (surfels_.empty())
    {
        return false;
    }
    for(int i = 0; i < surfels_.size(); i++)
    {
        pcl::PointCloud<Surfel> transform_result;
        pcl::transformPointCloud(surfels_[i], transform_result, poses_[i]);
        global_map += transform_result;
    }

    return true;
}

//bool
//SurfelMap::updateMap(const pcl::PointCloud<Surfel> &align_point,
//                     const pcl::CorrespondencesPtr mapping,
//                     int timestamp,
//                     std::shared_ptr<Point_2_Map> current_frame) {
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

bool SurfelMap::updateMap(int timestamp,
                     std::shared_ptr<Point_2_Map> current_frame) {
//    std::clock_t start_time = std::clock();
    // create a new timestamp surfel
    pcl::PointCloud<Surfel> new_time_clouds;
    surfels_.push_back(new_time_clouds);
    // transoform input local coordinate to world coordinate
    std::shared_ptr<Map_2_Point> transform_result = std::make_shared<Map_2_Point>(param_, initial_confidence_);
    pcl::transformPointCloud(current_frame->getPointClouds(), transform_result->setPointCloud(), poses_[timestamp]);
    // generate mapping index
    transform_result->generateMappingIndex();
    active_map_->generateMappingIndex();
    // get point
    pcl::PointCloud<Surfel>::Ptr origin_point = current_frame->getPointCloudsPtr();
    pcl::PointCloud<Surfel>::Ptr transform_point = transform_result->getPointCloudsPtr();
    int num_point = origin_point->size();
    std::vector<bool> updated_point(num_point, false);
    // update map
    for(int i = 0; i < num_point; i++)
    {
        int u = transform_result->getUIndex(i);
        int v = transform_result->getVIndex(i);
        int index = active_map_->getIndex(u, v);
        if(index == -1)
        {
            surfels_[timestamp].push_back(origin_point->points[i]);
        }
        else
        {
            // get nearest point data
            Eigen::Vector3d vsp({transform_point->points[i].x, transform_point->points[i].y, transform_point->points[i].z});
            Eigen::Vector3d nsp({transform_point->points[i].nx, transform_point->points[i].ny, transform_point->points[i].nz});
            float rsp = transform_point->points[i].radius;
            // get this point data
            Eigen::Vector3d vs({origin_point->points[i].x, origin_point->points[i].y, origin_point->points[i].z});
            Eigen::Vector3d ns({origin_point->points[i].nx, origin_point->points[i].ny, origin_point->points[i].nz});
            float rs = origin_point->points[i].radius;
            // compute reference
            float metric1 = std::abs(nsp.dot(vs - vsp));
            Eigen::Vector3d normal_ns_nsp = ns.cross(nsp);
            float metric2 = std::sqrt(normal_ns_nsp.dot(normal_ns_nsp));
        }
    }
    // generate kdtree
//    std::shared_ptr<pcl::KdTreeFLANN<Surfel, Surfel>> kdtree = std::make_shared<pcl::KdTreeFLANN<Surfel, Surfel>>();
////    pcl::KdTreeFLANN<Surfel, Surfel>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::KdTreeFLANN<Surfel, Surfel>>);
//    kdtree->setInputCloud(transform_result.makeShared());
//    std::vector<int> search_index;
//    std::vector<float> search_distance;
//
//    for(int i = 0; i < num_point; i++)
//    {
//        kdtree->nearestKSearch(origin_point->points[i], 1, search_index, search_distance);
//        if(search_distance[0] < max_distance_)
//        {
//            int active_map_index = search_index[0];
//            // get nearest point data
//            Eigen::Vector3d vsp({transform_result[active_map_index].x, transform_result[active_map_index].y, transform_result[active_map_index].z});
//            Eigen::Vector3d nsp({transform_result[active_map_index].nx, transform_result[active_map_index].ny, transform_result[active_map_index].nz});
//            float rsp = transform_result[active_map_index].radius;
//            // get this point data
//            Eigen::Vector3d vs({pointcloud->points[i].x, pointcloud->points[i].y, pointcloud->points[i].z});
//            Eigen::Vector3d ns({pointcloud->points[i].nx, pointcloud->points[i].ny, pointcloud->points[i].nz});
//            float rs = pointcloud->points[i].radius;
//            // compute reference
//            float metric1 = std::abs(nsp.dot(vs - vsp));
//            Eigen::Vector3d normal_ns_nsp = ns.cross(nsp);
//            float metric2 = std::sqrt(normal_ns_nsp.dot(normal_ns_nsp));
//            // verify threshold
//            if(metric1 < distance_thred_ && metric2 < angle_thred_ && rsp < rs)
//            {
//                // get active map index in surfel map
//                int origin_timestamp = timestamp - 1;
//                for(int i_2 = 0; i_2 < time_gap_; i_2++)
//                {
//
//                }
//            }
//        }
//        else
//        {
//            surfels_[timestamp].push_back(pointcloud->points[i]);
//        }
//    }

//    for(int i = 0; i < mapping->size(); i++)
//    {
//        int src_index = (*mapping)[i].index_query;
//        int tgt_index = (*mapping)[i].index_match;
//
//        updated_point[src_index] = true;
//        // get nearest point data
//        Eigen::Vector3d vsp({transform_result[tgt_index].x, transform_result[tgt_index].y, transform_result[tgt_index].z});
//        Eigen::Vector3d nsp({transform_result[tgt_index].nx, transform_result[tgt_index].ny, transform_result[tgt_index].nz});
//        float rsp = transform_result[tgt_index].radius;
//
//        Eigen::Vector3d vs({pointcloud->points[src_index].x, pointcloud->points[src_index].y, pointcloud->points[src_index].z});
//        Eigen::Vector3d ns({pointcloud->points[src_index].nx, pointcloud->points[src_index].ny, pointcloud->points[src_index].nz});
//        std::cout << "vs:" << vs << std::endl;
//        std::cout << "vsp:" << vsp << std::endl;
//        float rs = pointcloud->points[src_index].radius;
//        // compute reference
//        float metric1 = std::abs(nsp.dot(vs - vsp));
//        Eigen::Vector3d normal_ns_nsp = ns.cross(nsp);
//        float metric2 = std::sqrt(normal_ns_nsp.dot(normal_ns_nsp));
//        // verify threshold
//        if(metric1 < distance_thred_ && metric2 < angle_thred_ && rsp < rs)
//        {
//
////            pointcloud->points[src_index].x = (1 - gamma_) * pointcloud->points[src_index].x + gamma_ * transform_result[tgt_index].x;
////            pointcloud->points[src_index].y = (1 - gamma_) * pointcloud->points[src_index].y + gamma_ * transform_result[tgt_index].y;
////            pointcloud->points[src_index].z = (1 - gamma_) * pointcloud->points[src_index].z + gamma_ * transform_result[tgt_index].z;
////
////            pointcloud->points[src_index].nx = (1 - gamma_) * pointcloud->points[src_index].nx + gamma_ * transform_result[tgt_index].nx;
////            pointcloud->points[src_index].ny = (1 - gamma_) * pointcloud->points[src_index].ny + gamma_ * transform_result[tgt_index].ny;
////            pointcloud->points[src_index].nz = (1 - gamma_) * pointcloud->points[src_index].nz + gamma_ * transform_result[tgt_index].nz;
////
////            pointcloud->points[src_index].radius = pointcloud->points[src_index].radius;
//        }
//
//        else
//        {
////            surfels_[timestamp].push_back(pointcloud->points[src_index]);
//        }
//
//    }

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
    auto point_in_active_map = active_map_->setPointCloud();

    if (surfels_.size() == 0)
    {
        return false;
    }
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    // generate map
    for(int i = timestamp - 1, k = 0; i > timestamp - time_gap_ && i > -1; i--, k++)
    {
        pcl::PointCloud<Surfel> transform_result;
        pcl::transformPointCloud(surfels_[i], transform_result, poses_[i]);
        point_in_active_map += transform_result;
        active_map_index_in_map_[k] = transform_result.size();
    }
//    active_map_ += surfels_[timestamp - 1];
//    active_map_index_in_map_[0] = surfels_[timestamp - 1].size();
//    int k = 1;
//    for(int i = timestamp - 1; i > timestamp - time_gap_ && i > 0; i--)
//    {
//        pose = pose * poses_[i].inverse();
//        pcl::PointCloud<Surfel> transform_result;
//        pcl::transformPointCloud(surfels_[i - 1], transform_result, pose);
//        active_map_ += transform_result;
//        active_map_index_in_map_[k] = transform_result.size();
//    }
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