//
// Created by tongda on 2022/4/25.
//

#include "semanticmap.h"
#include "utils.h"
#include <pcl/common/transforms.h>

#define PCL_NO_PRECOMPILE

#include <pcl/kdtree/kdtree_flann.h>

SemanticMap::SemanticMap()
    : unactive_map_(new pcl::PointCloud<SemanticSurfel>), scManager_() {
  // initial vector
  global_poses_.resize(0);
  local_poses_.resize(0);
  semantic_map_.resize(0);

  global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map", 1000);

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

  surfel_sub_ =
      nh_.subscribe("surfel", 1000, &SemanticMap::SurfelCallback, this);
  pose_sub_ =
      nh_.subscribe("local_pose", 1000, &SemanticMap::PoseCallback, this);
  timer_ = nh_.createTimer(ros::Duration(5), &SemanticMap::PublishMap, this);

  current_frame_id_ = 0;
}

SemanticMap::~SemanticMap() {}

bool SemanticMap::UpdateMap(PoseType crt_pose) {
  // save pose
  SaveLocalPose(crt_pose);
  // create a new timestamp surfel
  pcl::PointCloud<SemanticSurfel>::Ptr new_pointcloud(
      new pcl::PointCloud<SemanticSurfel>());
  int num_point = current_point_cloud_->size();
  //    if (active_map_->points.size() > 0) {
  //        pcl::PointCloud<SemanticSurfel> odometry_result;
  //        pcl::transformPointCloud(*current_point_cloud_, odometry_result,
  //        crt_pose);
  //        // kdtree to find nearest point
  //        pcl::KdTreeFLANN<SemanticSurfel> kdtree;
  //        kdtree.setInputCloud(active_map_);
  //        int K = 1;
  //        std::vector<int> pointIdxKNNSearch(K);
  //        std::vector<float> pointKNNSquaredDistance(K);
  //        for (int i = 0; i < num_point; i++) {
  //            if (kdtree.nearestKSearch(odometry_result[i], K,
  //            pointIdxKNNSearch,
  //                                      pointKNNSquaredDistance) > 0) {
  //                if (pointKNNSquaredDistance[0] < 0.05 &&
  //                    active_map_->points[pointIdxKNNSearch[0]].label ==
  //                    current_point_cloud_->points[i].label) {
  //                    continue;
  //                }
  //            }
  //            new_pointcloud->push_back(current_point_cloud_->points[i]);
  //        }
  //    } else {
  for (int i = 0; i < num_point; i++) {
    new_pointcloud->push_back(current_point_cloud_->points[i]);
  }
  //    }
  semantic_map_.push_back(new_pointcloud);
  LoopDetection();
  current_frame_id_++;

  return true;
}

void SemanticMap::SaveLocalPose(PoseType crt_pose) {

  // save local speed
  local_poses_.push_back(crt_pose);
  // compute and save global pose
  PoseType global_pose;
  if (current_frame_id_ == 0) {
    global_pose = crt_pose;
  } else {
    global_pose = global_poses_[current_frame_id_ - 1] * crt_pose;
  }
  // svae global pose
  global_poses_.push_back(global_pose);
  // set optional graph node and edge
  factor_graph_.setInitialValues(current_frame_id_, global_pose);
  if (current_frame_id_ > 0) {
    factor_graph_.addEdge(current_frame_id_ - 1, current_frame_id_, crt_pose,
                          info_);
  }
}

bool SemanticMap::GenerateMap(pcl::PointCloud<SemanticSurfel>::Ptr global_map) {
  if (semantic_map_.empty()) {
    return false;
  }
  pcl::PointCloud<SemanticSurfel> transform_result;
  for (int i = 0; i < semantic_map_.size(); i++) {
    transform_result.clear();
    pcl::transformPointCloud(*semantic_map_[i], transform_result,
                             global_poses_[i]);
    *global_map += transform_result;
  }
  return true;
}

pcl::PointCloud<SemanticSurfel>::Ptr SemanticMap::GetUnActiveMapPtr(int id) {
  if (semantic_map_.empty()) {
    return nullptr;
  }
  int total_frame = semantic_map_.size();
  unactive_map_->clear();
  // generate map
  PoseType last_pose = PoseType::Identity();
  for (int i = id; i > id - time_gap_ && i > -1; i--) {
    pcl::PointCloud<SemanticSurfel> transform_result;
    pcl::transformPointCloud(*semantic_map_[i], transform_result, last_pose);
    *unactive_map_ += transform_result;
    last_pose = last_pose * local_poses_[i].inverse();
  }
  return unactive_map_;
}

bool SemanticMap::SetLoopSureEdge(int from, int to, PoseType &pose) {
  // optimise pose
  if (from - last_loop_index_ > detect_threshold_) {
    factor_graph_.addEdge(from, to, pose, info_);
    factor_graph_.optimize(30);
    factor_graph_.updatePoses(global_poses_);
    last_loop_index_ = from;
  }

  return true;
}

void SemanticMap::PublishMap(const ros::TimerEvent &event) {
  // generate global map
  pcl::PointCloud<SemanticSurfel>::Ptr global_map(
      new pcl::PointCloud<SemanticSurfel>);
  GenerateMap(global_map);
  // broadcast point numbers
  for (int i = 0; i < 10; i++) {
    std::cout << "*\t";
  }
  std::cout << std::endl;
  std::cout << "render map. SemanticMap have: " << global_map->size()
            << std::endl;
  for (int i = 0; i < 10; i++) {
    std::cout << "*\t";
  }
  std::cout << std::endl;
  // voxel filter
  FilterPointCloud(global_map, global_map, 0.5);
  // public to rviz
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*global_map, msg);
  msg.header.frame_id = "velodyne";
  global_map_pub_.publish(msg);
}

bool SemanticMap::LoopDetection() {
  scManager_.makeAndSaveScancontextAndKeys(*current_point_cloud_);
  auto pair_result = scManager_.detectLoopClosureID();
  int pre_index = pair_result.first;
  if (pre_index == -1 || pre_index > current_frame_id_) {
    //    if (!od_loopsure) {
    //      scManager_.popBack();
    //    }
    return false;
  }
  // get previous point cloud
  pcl::PointCloud<SemanticSurfel>::Ptr pre_point_clouds =
      GetUnActiveMapPtr(pre_index);
  // set initial_pose
  PoseType init_pose = PoseType::Identity();
  float yaw_arc = scManager_.getYawDiff();

  Eigen::AngleAxisd yawAngle(
      Eigen::AngleAxisd(yaw_arc, Eigen::Vector3d::UnitZ()));

  Eigen::Matrix3d rotation_matrix = yawAngle.toRotationMatrix();
  init_pose.block<3, 3>(0, 0) = rotation_matrix;

  PoseType res_pose =
      ComputePoseWithNdt(current_point_cloud_, pre_point_clouds, init_pose);
  std::cout << "pre index: " << pre_index
            << ", crt index: " << pair_result.second
            << " current in map: " << current_frame_id_ << std::endl;
  SetLoopSureEdge(current_frame_id_, pre_index, res_pose);
  return true;
}

void SemanticMap::SurfelCallback(const sensor_msgs::PointCloud2 &msg) {
  point_cloud_queue_.push(msg);
}

void SemanticMap::PoseCallback(const nav_msgs::Odometry &msg) {
  uint32_t current_seq_ = msg.header.seq;
  while (!point_cloud_queue_.empty()) {
    sensor_msgs::PointCloud2 point_cloud_msg = point_cloud_queue_.front();
    point_cloud_queue_.pop();
    if (point_cloud_msg.header.seq == current_seq_) {
      current_point_cloud_.reset(new pcl::PointCloud<SemanticSurfel>);
      pcl::fromROSMsg(point_cloud_msg, *current_point_cloud_);
      break;
    }
  }
  Eigen::Quaterniond q;
  q.x() = msg.pose.pose.orientation.x;
  q.y() = msg.pose.pose.orientation.y;
  q.z() = msg.pose.pose.orientation.z;
  q.w() = msg.pose.pose.orientation.w;

  Eigen::Vector3d t;
  t << msg.pose.pose.position.x, msg.pose.pose.position.y,
      msg.pose.pose.position.z;

  Eigen::Matrix4d current_pose(Eigen::Matrix4d::Identity());
  current_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  current_pose.block<3, 1>(0, 3) = t;
  UpdateMap(current_pose);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "semantic_map");
  SemanticMap semanticMap;
  ros::spin();
  return 0;
}