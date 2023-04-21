//
// Created by tongda on 2022/4/25.
//

#include "semanticmap.h"
#include <pcl/common/transforms.h>

#define PCL_NO_PRECOMPILE

#include <pcl/kdtree/kdtree_flann.h>

SemanticMap::SemanticMap()
        : active_map_(new pcl::PointCloud<Surfel>),
          unactive_map_(new pcl::PointCloud<Surfel>) {
    // initial vector
    global_poses_.resize(0);
    local_poses_.resize(0);
    semantic_map_.resize(0);

    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map", 1000);
    global_path_pub_ = nh_.advertise<nav_msgs::Path>("gloabl_path", 1000);

    auto &diag = info_.diagonal();
    // translational noise is smaller than ro tational noise.
    double transNoise = 1.0;
    double rotNoise = 1.0;
    diag[0] = (transNoise * transNoise);
    diag[1] = (transNoise * transNoise);
    diag[2] = (transNoise * transNoise);

    diag[3] = (rotNoise * rotNoise);
    diag[4] = (rotNoise * rotNoise);
    diag[5] = (rotNoise * rotNoise);

    nh_.getParam("work_directory", work_directory_);
    nh_.getParam("detection_treshold", detect_threshold_);
    nh_.getParam("is_loop_detection", loop_closure_detection_);
    nh_.getParam("pose_out_path", pose_out_path_);
    nh_.getParam("time_gap", time_gap_);
    pose_out_path_ = work_directory_ + pose_out_path_;

    current_frame_id_ = 0;
}

SemanticMap::~SemanticMap() {}

pcl::PointCloud<Surfel>::Ptr SemanticMap::getActiveMapPtr() {
    if (semantic_map_.empty()) {
        return nullptr;
    }
    int total_frame = semantic_map_.size();
    active_map_->clear();
    // generate map
    pose_type last_pose = pose_type::Identity();
    for (int i = total_frame - 1; i > total_frame - time_gap_ && i > -1; i--) {
        pcl::PointCloud<Surfel> transform_result;
        pcl::transformPointCloud(*semantic_map_[i], transform_result, last_pose);
        *active_map_ += transform_result;
        last_pose = last_pose * local_poses_[i].inverse();
    }
    return active_map_;
}

bool SemanticMap::updateMap(pcl::PointCloud<Surfel>::Ptr current_frame,
                            pose_type crt_pose) {
    // save pose
    saveLocalPose(crt_pose);
    // create a new timestamp surfel
    pcl::PointCloud<Surfel>::Ptr new_pointcloud(new pcl::PointCloud<Surfel>());
    int num_point = current_frame->size();
    if (active_map_->points.size() > 0) {
        pcl::PointCloud<Surfel> odometry_result;
        pcl::transformPointCloud(*current_frame, odometry_result, crt_pose);
        // kdtree to find nearest point
        pcl::KdTreeFLANN<Surfel> kdtree;
        kdtree.setInputCloud(active_map_);
        int K = 1;
        std::vector<int> pointIdxKNNSearch(K);
        std::vector<float> pointKNNSquaredDistance(K);
        for (int i = 0; i < num_point; i++) {
            if (kdtree.nearestKSearch(odometry_result[i], K, pointIdxKNNSearch,
                                      pointKNNSquaredDistance) > 0) {
                if (pointKNNSquaredDistance[0] < 0.05 &&
                    active_map_->points[pointIdxKNNSearch[0]].point_type ==
                    current_frame->points[i].point_type) {
                    continue;
                }
            }
            new_pointcloud->push_back(current_frame->points[i]);
        }
    } else {
        for (int i = 0; i < num_point; i++) {
            new_pointcloud->push_back(current_frame->points[i]);
        }
    }
    semantic_map_.push_back(new_pointcloud);
    current_frame_id_++;

    return true;
}

void SemanticMap::saveLocalPose(pose_type crt_pose) {

    // save local speed
    local_poses_.push_back(crt_pose);
    // compute and save global pose
    pose_type global_pose;
    if (current_frame_id_ == 0) {
        global_pose = crt_pose;
    } else {
        global_pose = global_poses_[current_frame_id_ - 1] * crt_pose;
    }
    // svae global pose
    global_poses_.push_back(global_pose);
    // set optional graph node and edge
    factor_graph_.setInitialValues(current_frame_id_, global_pose.cast<double>());
    if (current_frame_id_ > 0) {
        factor_graph_.addEdge(current_frame_id_ - 1, current_frame_id_,
                              crt_pose.cast<double>(), info_);
    }

    geometry_msgs::PoseStamped pose_tmp;

    pose_tmp.header.frame_id = "velodyne";
    pose_tmp.header.stamp = ros::Time::now();
    pose_tmp.pose.position.x = global_pose(0, 3);
    pose_tmp.pose.position.y = global_pose(1, 3);
    pose_tmp.pose.position.z = global_pose(2, 3);
}

pose_type &SemanticMap::getLastPose() { return local_poses_.back(); }

bool SemanticMap::generateMap(pcl::PointCloud<Surfel>::Ptr global_map) {
    if (semantic_map_.empty()) {
        return false;
    }
    pcl::PointCloud<Surfel> transform_result;
    for (int i = 0; i < semantic_map_.size(); i++) {
        transform_result.clear();
        pcl::transformPointCloud(*semantic_map_[i], transform_result,
                                 global_poses_[i]);
        *global_map += transform_result;
    }
    return true;
}

int SemanticMap::getCurrentFrameId() { return current_frame_id_; }

pcl::PointCloud<Surfel>::Ptr SemanticMap::getPointCloudsInLocal(int id) {
    return semantic_map_[id];
}

pcl::PointCloud<Surfel>::Ptr SemanticMap::getUnActiveMapPtr(int id) {
    if (semantic_map_.empty()) {
        return nullptr;
    }
    int total_frame = semantic_map_.size();
    unactive_map_->clear();
    // generate map
    pose_type last_pose = pose_type::Identity();
    for (int i = id; i > id - time_gap_ && i > -1; i--) {
        pcl::PointCloud<Surfel> transform_result;
        pcl::transformPointCloud(*semantic_map_[i], transform_result, last_pose);
        *unactive_map_ += transform_result;
        last_pose = last_pose * local_poses_[i].inverse();
    }
    return unactive_map_;
}

bool SemanticMap::setLoopsureEdge(int from, int to, pose_type &pose) {
    // optimise pose
    if (from - last_loop_index_ > detect_threshold_) {
        factor_graph_.addEdge(from, to, pose.cast<double>(), info_);
        factor_graph_.optimize(30);
        factor_graph_.updatePoses(global_poses_);
        last_loop_index_ = from;
    }

    return true;
}