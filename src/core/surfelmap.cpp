//
// Created by tongda on 2021/12/19.
//

#include "surfelmap.h"
const float PI = acos(-1);


SurfelMap::SurfelMap(rv::ParameterList parameter_list, std::shared_ptr<Timestamp> time):
        global_poses_(),
        local_poses_(),
        surfel_map_(0),
        p_stable_(parameter_list["p_stable"]),
        p_prior_(parameter_list["p_prior"]),
        sigma_angle_2_(parameter_list["sigma_angle"]),
        sigma_distance_2_(parameter_list["sigma_distance"]),
        distance_thred_(parameter_list["distance_thred"]),
        angle_thred_(parameter_list["angle_thred"]),
        gamma_(parameter_list["gamma"]),
        confidence_thred_(parameter_list["confidence_threshold"]),
        time_gap_(parameter_list["time_gap"]),
        active_map_index_in_map_(time_gap_, -1),
        pose_graph_(),
        timestamp_(time),
        loop_edges_()
{
    odds_p_prior_ = std::log(p_prior_ / (1 - p_prior_));
    initial_confidence_ = std::log(p_stable_ / (1 - p_stable_)) - odds_p_prior_;
    sigma_angle_2_ = sigma_angle_2_ * sigma_angle_2_;
    sigma_distance_2_ = sigma_distance_2_ * sigma_distance_2_;
    angle_thred_ = std::sin(angle_thred_);
    active_map_ = std::make_shared<VertexMap>(parameter_list, initial_confidence_);
    param_ = parameter_list;
    loop_thred_ = param_["loop_thred"];
    have_loop_ = false;

    auto& diag = info_.diagonal();
    // translational noise is smaller than rotational noise.
    float transNoise = 1.0;
    float rotNoise = 1.0;
    diag[0] = (transNoise * transNoise);
    diag[1] = (transNoise * transNoise);
    diag[2] = (transNoise * transNoise);

    diag[3] = (rotNoise * rotNoise);
    diag[4] = (rotNoise * rotNoise);
    diag[5] = (rotNoise * rotNoise);

    cam2velo_ << 0, 0, 1, 0,
            -1, 0, 0, 0,
            0, -1, 0, 0.08,
            0, 0, 0, 1;
    velo2cam_ << 0, -1, 0, 0,
            0, 0, -1, 0,
            1, 0, 0, -0.08,
            0, 0, 0, 1;

    cam2velo_ = cam2velo_.inverse().eval();
    velo2cam_ = velo2cam_.inverse().eval();
}

SurfelMap::~SurfelMap() {

}

bool SurfelMap::pushBackPose(pose_type& pose) {
    // save local speed
    local_poses_.push_back(pose);
    // get id
    int id = global_poses_.size();
    // compute and save global pose
    pose_type global_pose;
    if(id == 0)
    {
        global_pose = pose;
    }
    else
    {
        global_pose = global_poses_[id - 1] * pose;
    }
    global_poses_.push_back(global_pose);

    // set optional graph node and edge
    pose_graph_.setInitialValues(id, global_pose.cast<double>());
    if(id > 0)
    {
        pose_graph_.addEdge(id - 1, id, pose.cast<double>(), info_);
    }
    return true;
}

pose_type SurfelMap::getLastPose() {
    return local_poses_.back();
}

bool SurfelMap::mapInitial(pose_type init_pose)
{
    // push init pose
    pushBackPose(init_pose);
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
    if (surfel_map_.empty())
    {
        return false;
    }
//    removeUnstableSurfel();
    pcl::PointCloud<Surfel> transform_result;
    for(int i = 0; i < surfel_map_.size(); i++)
    {
        transform_result.clear();
        pcl::transformPointCloud(*surfel_map_[i], transform_result, global_poses_[i]);
        global_map += transform_result;
    }
//    for(int i = surfel_map_.size() - 1; i > -1; i--)
//    {
//        global_map += *surfel_map_[i];
//        pcl::transformPointCloud(global_map, global_map, local_poses_[i]);
//    }

    return true;
}

//bool SurfelMap::updateMap(std::shared_ptr<VertexMap> current_frame) {
//    int now_time = timestamp_->getCurrentime();
//    // create a new timestamp surfel
//    std::shared_ptr<pcl::PointCloud<Surfel>> new_time_clouds = std::make_shared<pcl::PointCloud<Surfel>>();
//    surfel_map_.push_back(new_time_clouds);
//    // if timestamp = 0, only insert point
//    if(now_time == 0)
//    {
//        std::shared_ptr<pcl::PointCloud<Surfel>> origin_point = current_frame->getPointCloudsPtr();
//        int num_point = origin_point->size();
//        for(int i = 0; i < num_point; i++)
//        {
//            surfel_map_[now_time]->push_back(origin_point->points[i]);
//        }
//        return true;
//    }
//    std::shared_ptr<PointIndex> transform_result = std::make_shared<PointIndex>(param_);
//    // get point
//    std::shared_ptr<pcl::PointCloud<Surfel>> origin_point = current_frame->getPointCloudsPtr();
//    std::shared_ptr<pcl::PointCloud<Surfel>>  transform_point = transform_result->getPointCloudsPtr();
//    std::shared_ptr<pcl::PointCloud<Surfel>> active_map_point = active_map_->getPointCloudsPtr();
//    // transoform input local coordinate to world coordinate
//    pcl::transformPointCloud(*current_frame->getPointCloudsPtr(), *transform_point, global_poses_[now_time]);
//    // generate mapping index
//    transform_result->generateMappingIndex();
//    active_map_->generateMappingIndex();
//    int num_point = origin_point->size();
//    std::vector<bool> updated_point(num_point, false);
//    // update map
////    for(int i = 0; i < num_point; i++)
////    {
////        int u = transform_result->getUIndex(i);
////        int v = transform_result->getVIndex(i);
////        if(u < 0 || v < 0 || u >= width_ || v >= height_)
////        {
////            surfel_map_[timestamp]->push_back(origin_point->points[i]);
////            continue;
////        }
////        int index = active_map_->getIndex(u, v);
////        if(index == -1)
////        {
////            surfel_map_[timestamp]->push_back(origin_point->points[i]);
////            continue;
////        }
////        else
////        {
////            // get matching point data
////            Eigen::Vector3d vsp({active_map_point->points[index].x, active_map_point->points[index].y, active_map_point->points[index].z});
////            Eigen::Vector3d nsp({active_map_point->points[index].nx, active_map_point->points[index].ny, active_map_point->points[index].nz});
////            float rsp = active_map_point->points[index].radius;
////            // get this point data
////            Eigen::Vector3d vs({transform_point->points[i].x, transform_point->points[i].y, transform_point->points[i].z});
////            Eigen::Vector3d ns({transform_point->points[i].nx, transform_point->points[i].ny, transform_point->points[i].nz});
////            float rs = transform_point->points[i].radius;
////            // compute reference
////            float metric1 = std::abs(nsp.dot(vs - vsp));
////            Eigen::Vector3d normal_ns_nsp = ns.cross(nsp);
////            float metric2 = std::sqrt(normal_ns_nsp.dot(normal_ns_nsp));
////            // compute origin point position
////            int match_point_time = 0;
////            while(index > active_map_index_in_map_[match_point_time])
////            {
////                index -= active_map_index_in_map_[match_point_time];
////                match_point_time++;
////            }
////            match_point_time = timestamp - match_point_time - 1;
////            float orgin_confidence = surfel_map_[match_point_time]->points[index].confidence;
////            // compute angle and distance error
////            float angle_cos = ns.dot(nsp) / (ns.norm() * nsp.norm());
////            float angle_2 = acos(angle_cos);
////            angle_2 = angle_2 * angle_2;
////            float dis_2 = (vs - vsp).squaredNorm();
////            float update_confidence = updateConfidence(orgin_confidence, angle_2, dis_2);
////            // update confidence
////            surfel_map_[match_point_time]->points[index].confidence = update_confidence;
////            if(metric1 < max_distance && metric2 < angle_thred_ && rs < rsp)
////            {
////                // transform point to local coordinate
////                Eigen::Isometry3f local_pose(global_poses_[match_point_time]);
////                Eigen::Vector3f origin_xyz({transform_point->points[i].x, transform_point->points[i].y, transform_point->points[i].z});
////                Eigen::Vector3f transform_xyz = local_pose.inverse() * origin_xyz;
////                surfel_map_[match_point_time]->points[index].x = (1 - gamma_) * transform_xyz.x() +
////                        gamma_ * surfel_map_[match_point_time]->points[index].x;
////                surfel_map_[match_point_time]->points[index].x = (1 - gamma_) * transform_xyz.y() +
////                                                             gamma_ * surfel_map_[match_point_time]->points[index].y;
////                surfel_map_[match_point_time]->points[index].x = (1 - gamma_) * transform_xyz.z() +
////                                                             gamma_ * surfel_map_[match_point_time]->points[index].z;
////                surfel_map_[match_point_time]->points[index].nx = (1 - gamma_) * origin_point->points[i].nx +
////                                                             gamma_ * surfel_map_[match_point_time]->points[index].nx;
////                surfel_map_[match_point_time]->points[index].ny = (1 - gamma_) * origin_point->points[i].ny +
////                                                              gamma_ * surfel_map_[match_point_time]->points[index].ny;
////                surfel_map_[match_point_time]->points[index].nz = (1 - gamma_) * origin_point->points[i].nz +
////                                                              gamma_ * surfel_map_[match_point_time]->points[index].nz;
////                surfel_map_[match_point_time]->points[index].radius = origin_point->points[i].radius;
////            }
////            else
////            {
////                surfel_map_[timestamp]->push_back(origin_point->points[i]);
////            }
////        }
////    }
//
//    for(int i = 0; i < num_point; i++)
//    {
//        if(updated_point[i])
//        {
//            continue;
//        }
//        surfel_map_[now_time]->push_back(origin_point->points[i]);
//    }
//    return true;
//}

bool SurfelMap::updateMap(std::shared_ptr<VertexMap> current_frame, bool mode, pose_type crt_pose)
{
    int now_time = timestamp_->getCurrentime();
    // if it is a key frame, we insert it directly
    if(mode)
    {
        // create a new timestamp surfel
        std::shared_ptr<pcl::PointCloud<Surfel>> new_time_clouds = std::make_shared<pcl::PointCloud<Surfel>>();
        surfel_map_.push_back(new_time_clouds);
        pushBackPose(crt_pose);
        std::shared_ptr<pcl::PointCloud<Surfel>> origin_point = current_frame->getPointCloudsPtr();
        int num_point = origin_point->size();
        for(int i = 0; i < num_point; i++)
        {
            new_time_clouds->push_back(origin_point->points[i]);
        }
        return true;
    }

//    std::shared_ptr<PointIndex> transform_result = std::make_shared<PointIndex>(param_);
//    // get point
//    std::shared_ptr<pcl::PointCloud<Surfel>> origin_point = current_frame->getPointCloudsPtr();
//    std::shared_ptr<pcl::PointCloud<Surfel>>  transform_point = transform_result->getPointCloudsPtr();
//    std::shared_ptr<pcl::PointCloud<Surfel>> active_map_point = active_map_->getPointCloudsPtr();
//    // transoform input local coordinate to world coordinate
//    pcl::transformPointCloud(*current_frame->getPointCloudsPtr(), *transform_point, global_poses_[now_time]);
//    // generate mapping index
//    transform_result->generateMappingIndex();
//    active_map_->generateMappingIndex();
//    int num_point = origin_point->size();
//    std::vector<bool> updated_point(num_point, false);
    // update map


//    for(int i = 0; i < num_point; i++)
//    {
//        if(updated_point[i])
//        {
//            continue;
//        }
//        surfel_map_[now_time]->push_back(origin_point->points[i]);
//    }
    return true;
}
bool SurfelMap::generateActiveMap()
{
    int total_frame = surfel_map_.size();
    std::shared_ptr<pcl::PointCloud<Surfel>> point_in_active_map = active_map_->getPointCloudsPtr();
    point_in_active_map->clear();

    if (surfel_map_.empty())
    {
        return false;
    }
    // generate map
    pose_type last_pose = pose_type::Identity();
    for(int i = total_frame - 1, k = 0; i > total_frame - time_gap_ && i > -1; i--, k++)
    {
        pcl::PointCloud<Surfel> transform_result;
        pcl::transformPointCloud(*surfel_map_[i], transform_result, last_pose);
        *point_in_active_map += transform_result;
        active_map_index_in_map_[k] = transform_result.size();
        last_pose = last_pose * local_poses_[i].inverse();
    }
    return true;
}

bool SurfelMap::removeUnstableSurfel() {
    for(int i = 0; i < surfel_map_.size(); i++)
    {
        int num_points = surfel_map_[i]->size();
        std::shared_ptr<pcl::PointCloud<Surfel>> point_clouds = surfel_map_[i];
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

std::shared_ptr<pcl::PointCloud<Surfel>> SurfelMap::getPointCloudsInLocal(int timestamp)
{
    return surfel_map_[timestamp];
}

std::shared_ptr<pcl::PointCloud<Surfel>> SurfelMap::getPointCloudsInGlobal(int timestamp)
{
    std::shared_ptr<pcl::PointCloud<Surfel>> transform_result = std::make_shared<pcl::PointCloud<Surfel>>();
    pcl::transformPointCloud(*surfel_map_[timestamp], *transform_result, global_poses_[timestamp]);
    return transform_result;
}

bool SurfelMap::setLoopsureEdge(int from, int to, pose_type& pose) {
    // valid last loopsure
    if(have_loop_ && from - loop_edges_.from < loop_thred_)
    {
        // optimise pose
        std::cout << "1 to" << std::endl;
        pose_graph_.addEdge(loop_edges_.from, loop_edges_.to, loop_edges_.pose.cast<double>(), info_);
        std::cout << "2 to" << std::endl;
        pose_graph_.optimize(30);
        std::cout << "3 to" << std::endl;
        pose_graph_.updatePoses(global_poses_);
        std::cout << "4 to" << std::endl;
    }
    std::cout << "setting" << std::endl;
    loop_edges_.from = from;
    loop_edges_.to = to;
    std::cout << "end setting" << std::endl;
    loop_edges_.pose = pose;
    have_loop_ = true;

    return true;
}

bool SurfelMap::resetLoopTimes()
{
    have_loop_ = false;
    return true;
}

bool SurfelMap::savePose2File()
{
    std::ofstream filehandle;
    filehandle.open(param_["pose_path"]);
    if(filehandle.is_open())
    {
        for(int i = 0; i < global_poses_.size(); i++)
        {
            // transfer pose to camera coordinate
            global_poses_[i] = cam2velo_ * global_poses_[i] * velo2cam_;
            // save pose as kitti style
            for(int j = 0; j < 11; j++)
            {
                int u = j / 4;
                int v = j % 4;
                filehandle << global_poses_[i](u, v) << " ";
//                filehandle << local_poses_[i](u, v) << " ";
            }
            filehandle << global_poses_[i](3, 3) << std::endl;
//            filehandle << local_poses_[i](3, 3) << std::endl;
        }
        filehandle.close();
    }
    else
    {
        filehandle.close();
        return false;
    }
    return true;
}