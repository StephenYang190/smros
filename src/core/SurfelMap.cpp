//
// Created by tongda on 2021/12/19.
//

#include "SurfelMap.h"
const float PI = acos(-1);


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

bool SurfelMap::mapInitial(std::shared_ptr<pcl::PointCloud<Surfel>> pointcloud, Eigen::Matrix4f init_pose)
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
//    removeUnstableSurfel();
    for(int i = 0; i < surfels_.size(); i++)
    {
        pcl::PointCloud<Surfel> transform_result;
        pcl::transformPointCloud(*surfels_[i], transform_result, poses_[i]);
        global_map += transform_result;
    }

    return true;
}

bool SurfelMap::updateMap(int timestamp,
                     std::shared_ptr<Point_2_Map> current_frame) {
//    std::clock_t start_time = std::clock();
    // create a new timestamp surfel
    std::shared_ptr<pcl::PointCloud<Surfel>> new_time_clouds = std::make_shared<pcl::PointCloud<Surfel>>();
    surfels_.push_back(new_time_clouds);
    // transoform input local coordinate to world coordinate
    std::shared_ptr<Map_2_Point> transform_result = std::make_shared<Map_2_Point>(param_, initial_confidence_);
    pcl::transformPointCloud(*current_frame->getPointCloudsPtr(), transform_result->setPointCloud(), poses_[timestamp]);
    // generate mapping index
    transform_result->generateMappingIndex();
    active_map_->generateMappingIndex();
    // get point
    std::shared_ptr<pcl::PointCloud<Surfel>> origin_point = current_frame->getPointCloudsPtr();
    pcl::PointCloud<Surfel>::Ptr transform_point = transform_result->getPointCloudsPtr();
    std::shared_ptr<pcl::PointCloud<Surfel>> active_map_point = active_map_->getPointCloudsPtr();
    int num_point = origin_point->size();
    std::vector<bool> updated_point(num_point, false);
    // update map
    for(int i = 0; i < num_point; i++)
    {
        int u = transform_result->getUIndex(i);
        int v = transform_result->getVIndex(i);
        if(u < 0 || v < 0 || u >= width_ || v >= height_)
        {
            surfels_[timestamp]->push_back(origin_point->points[i]);
            continue;
        }
        int index = active_map_->getIndex(u, v);
        if(index == -1)
        {
//            surfels_[timestamp].push_back(origin_point->points[i]);
            surfels_[timestamp]->push_back(origin_point->points[i]);
            continue;
        }
        else
        {
            // get matching point data
            Eigen::Vector3f vsp({active_map_point->points[index].x, active_map_point->points[index].y, active_map_point->points[index].z});
            Eigen::Vector3f nsp({active_map_point->points[index].nx, active_map_point->points[index].ny, active_map_point->points[index].nz});
            float rsp = active_map_point->points[index].radius;
            // get this point data
            Eigen::Vector3f vs({transform_point->points[i].x, transform_point->points[i].y, transform_point->points[i].z});
            Eigen::Vector3f ns({transform_point->points[i].nx, transform_point->points[i].ny, transform_point->points[i].nz});
            float rs = transform_point->points[i].radius;
            // compute reference
            float metric1 = std::abs(nsp.dot(vs - vsp));
            Eigen::Vector3f normal_ns_nsp = ns.cross(nsp);
            float metric2 = std::sqrt(normal_ns_nsp.dot(normal_ns_nsp));
            // compute origin point position
            int match_point_time = 0;
            while(index > active_map_index_in_map_[match_point_time])
            {
                index -= active_map_index_in_map_[match_point_time];
                match_point_time++;
            }
            match_point_time = timestamp - match_point_time - 1;
            float orgin_confidence = surfels_[match_point_time]->points[index].confidence;
            // compute angle and distance error
            float angle_cos = ns.dot(nsp) / (ns.norm() * nsp.norm());
            float angle_2 = acos(angle_cos);
            angle_2 = angle_2 * angle_2;
            float dis_2 = (vs - vsp).squaredNorm();
            float update_confidence = updateConfidence(orgin_confidence, angle_2, dis_2);
            // update confidence
            surfels_[match_point_time]->points[index].confidence = update_confidence;
            if(metric1 < distance_thred_ && metric2 < angle_thred_ && rs < rsp)
            {
                // transform point to local coordinate
                Eigen::Isometry3f local_pose(poses_[match_point_time]);
                Eigen::Vector3f origin_xyz({transform_point->points[i].x, transform_point->points[i].y, transform_point->points[i].z});
                Eigen::Vector3f transform_xyz = local_pose.inverse() * origin_xyz;
                surfels_[match_point_time]->points[index].x = (1 - gamma_) * transform_xyz.x() +
                        gamma_ * surfels_[match_point_time]->points[index].x;
                surfels_[match_point_time]->points[index].x = (1 - gamma_) * transform_xyz.y() +
                                                             gamma_ * surfels_[match_point_time]->points[index].y;
                surfels_[match_point_time]->points[index].x = (1 - gamma_) * transform_xyz.z() +
                                                             gamma_ * surfels_[match_point_time]->points[index].z;
                surfels_[match_point_time]->points[index].nx = (1 - gamma_) * origin_point->points[i].nx +
                                                             gamma_ * surfels_[match_point_time]->points[index].nx;
                surfels_[match_point_time]->points[index].ny = (1 - gamma_) * origin_point->points[i].ny +
                                                              gamma_ * surfels_[match_point_time]->points[index].ny;
                surfels_[match_point_time]->points[index].nz = (1 - gamma_) * origin_point->points[i].nz +
                                                              gamma_ * surfels_[match_point_time]->points[index].nz;
                surfels_[match_point_time]->points[index].radius = origin_point->points[i].radius;
            }
            else
            {
                surfels_[timestamp]->push_back(origin_point->points[i]);
            }
        }
    }

//    for(int i = 0; i < num_point; i++)
//    {
//        if(updated_point[i])
//        {
//            continue;
//        }
//        surfels_[timestamp].push_back(origin_point->points[i]);
//    }

    return true;
}

bool SurfelMap::generateActiveMap(int timestamp)
{
    std::shared_ptr<pcl::PointCloud<Surfel>> point_in_active_map = active_map_->setPointCloud();

    if (surfels_.size() == 0)
    {
        return false;
    }
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    // generate map
    for(int i = timestamp - 1, k = 0; i > timestamp - time_gap_ && i > -1; i--, k++)
    {
        pcl::PointCloud<Surfel> transform_result;
        pcl::transformPointCloud(*surfels_[i], transform_result, poses_[i]);
        *point_in_active_map += transform_result;
        active_map_index_in_map_[k] = transform_result.size();
    }
    return true;
}

bool SurfelMap::removeUnstableSurfel() {
    for(int i = 0; i < surfels_.size(); i++)
    {
        int num_points = surfels_[i]->size();
        std::shared_ptr<pcl::PointCloud<Surfel>> point_clouds = surfels_[i];
        for(int i_2 = 0; i_2 < num_points; i_2++)
        {
            if(point_clouds->points[i_2].confidence < confidence_thred_)
            {
                point_clouds->points[i_2].x = NAN;
            }
        }
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*point_clouds, *point_clouds, mapping);
    }
    return false;
}
