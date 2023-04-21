//
// Created by tongdayang on 1/12/23.
//

#include "localization.h"

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include "utils.h"

const float PI = acos(-1);

Localization::Localization()
    : current_point_cloud_(new pcl::PointCloud<Surfel>), scManager_() {
  nh_.getParam("max_distance_gap", max_distance_);
  nh_.getParam("max_angle_gap", max_angle_);
  local_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("local_pose", 1000);
  global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map", 1000);
  current_pose = Eigen::Matrix4f::Identity();
  surfel_sub_ =
      nh_.subscribe("surfel", 1000, &Localization::SurfelCallback, this);
  timer_ = nh_.createTimer(ros::Duration(5), &Localization::publishMap, this);
}

Localization::~Localization() {}

void Localization::SurfelCallback(const sensor_msgs::PointCloud2 &msg) {
  pcl::fromROSMsg(msg, *current_point_cloud_);
  current_seq_ = msg.header.seq;
  current_frame_id_ = map_.getCurrentFrameId();
  laserOdometry();
  map_.updateMap(current_point_cloud_, current_pose);
  loopDetection();
}

void Localization::publishMap(const ros::TimerEvent &event) {
  // generate global map
  pcl::PointCloud<Surfel>::Ptr global_map(new pcl::PointCloud<Surfel>);
  map_.generateMap(global_map);
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

void Localization::laserOdometry() {
  auto active_map = map_.getActiveMapPtr();
  if (active_map) {
    auto last_pose = map_.getLastPose();
    current_pose = ComputePoseWithNonlinear(current_point_cloud_, active_map, last_pose);
    Eigen::Matrix3f rotation = current_pose.block<3, 3>(0, 0);
    // TODO: detect t matrix index 3, 1? or 1, 3?
    Eigen::Vector3f t(current_pose.block<3, 1>(0, 3));
    float distance = t.norm();
    float yaw_angle = rotation.eulerAngles(2, 1, 0)[0] / PI * 180;
    //    std::cout << "yaw angle : " << yaw_angle << std::endl;
    //    std::cout << "distance between two frames is : " << distance <<
    //    std::endl;
  }
  // transfer matrix to quaternion
  Eigen::Quaternionf q(current_pose.block<3, 3>(0, 0));
  Eigen::Vector3f t(current_pose.block<1, 3>(0, 3));
  // publish pose
  nav_msgs::Odometry current_odometry;
  current_odometry.header.frame_id = "velodyne";
  current_odometry.child_frame_id = "laser_odom";
  current_odometry.header.seq = current_seq_;
  current_odometry.pose.pose.orientation.x = q.x();
  current_odometry.pose.pose.orientation.y = q.y();
  current_odometry.pose.pose.orientation.z = q.z();
  current_odometry.pose.pose.orientation.w = q.w();
  current_odometry.pose.pose.position.x = t.x();
  current_odometry.pose.pose.position.y = t.y();
  current_odometry.pose.pose.position.z = t.z();
  local_pose_pub_.publish(current_odometry);
}

bool Localization::loopDetection() {
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
  pcl::PointCloud<Surfel>::Ptr pre_point_clouds =
      map_.getUnActiveMapPtr(pre_index);
  // set initial_pose
  pose_type init_pose = pose_type::Identity();
  float yaw_arc = scManager_.getYawDiff();
  
  Eigen::AngleAxisf yawAngle(
      Eigen::AngleAxisd(yaw_arc, Eigen::Vector3d::UnitZ()));

  Eigen::Matrix3f rotation_matrix = yawAngle.toRotationMatrix();
  init_pose.block<3, 3>(0, 0) = rotation_matrix;

  pose_type res_pose =
          ComputePoseWithNdt(current_point_cloud_, pre_point_clouds, init_pose);
  std::cout << "pre index: " << pre_index
            << ", crt index: " << pair_result.second
            << " current in map: " << current_frame_id_ << std::endl;
  map_.setLoopsureEdge(current_frame_id_, pre_index, res_pose);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "localization");
  Localization localization;
  ros::spin();
  return 0;
}