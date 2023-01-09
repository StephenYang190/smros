//
// Created by tongda on 2022/4/25.
//

#ifndef SRC_SMTEST_H
#define SRC_SEMANTICMAP_H

#define PCL_NO_PRECOMPILE

#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_ros/point_cloud.h>
#include <queue>

#include "backendopt.h"
#include "smros_msgs/Keyframe.h"
#include "surfel.h"

class SemanticMap {
public:
  SemanticMap();
  // add pose to pose list
  void OdometryCallback(const nav_msgs::Odometry &msg);
  void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void KeyframeCallback(const smros_msgs::Keyframe &msg);

protected:
private:
  // store pose frame by frame
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      global_poses_;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      local_poses_;
  std::queue<nav_msgs::Odometry> tmp_local_poses_;
  // store point clouds frame by frame
  std::vector<pcl::PointCloud<Surfel>::Ptr> semantic_map_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> tmp_point_cloud_;
  // ros nodehandle
  ros::NodeHandle nh_;
  std::string work_directory_;
  // ros publisher
  ros::Publisher global_map_pub_;
  ros::Publisher global_path_pub_;
  // pose graph
  BackEndOpt pose_graph_;
  // information metrix
  Eigen::DiagonalMatrix<double, 6> info_;
  // parameter used to control the time to optimization
  int detect_threshold_;
  // parameter used to mark loop closure
  bool loop_closure_detection_;
  // path to save pose
  std::string pose_out_path_;
  // timestamp
  int timestamp_;
  // navigation path message
  nav_msgs::Path global_path_;
  // mutex to control multi thread
  std::mutex global_pose_mutex_;
  std::mutex tmp_point_cloud_mutex_;
  std::mutex tmp_pose_mutex_;

  ros::Subscriber point_cloud_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber keyframe_sub_;
};

#endif // SRC_SMTEST_H
