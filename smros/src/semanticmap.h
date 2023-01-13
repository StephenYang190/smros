//
// Created by tongda on 2022/4/25.
//

#ifndef SRC_SMTEST_H
#define SRC_SEMANTICMAP_H

#define PCL_NO_PRECOMPILE

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_ros/point_cloud.h>
#include <queue>

#include "backendopt.h"
#include "surfel.h"

using pose_type = Eigen::Matrix4f;
class SemanticMap {
public:
  SemanticMap();
  ~SemanticMap();
  // get active map ptr
  pcl::PointCloud<Surfel>::Ptr getActiveMapPtr();
  // update map
  bool updateMap(pcl::PointCloud<Surfel>::Ptr current_frame,
                 pose_type crt_pose = pose_type::Identity());
  void saveLocalPose(pose_type crt_pose);
  // get pose at timestamp
  pose_type &getLastPose();
  // generate global map
  bool generateMap(pcl::PointCloud<Surfel>::Ptr global_map);
  // get final point cloud index
  int getCurrentFrameId();
  // get point clouds at id in local coordination
  pcl::PointCloud<Surfel>::Ptr getPointCloudsInLocal(int id);
  pcl::PointCloud<Surfel>::Ptr getUnActiveMapPtr(int id);
  // set loop edge in factor graph
  bool setLoopsureEdge(int from, int to, pose_type &pose);
  // get point clouds at id in global coordination
  pcl::PointCloud<Surfel>::Ptr getPointCloudsInGlobal(int id);
  // reset loop times
  bool resetLoopTimes();
  // sae pose to file
  bool savePose2File(std::string suffix);
  // ros path
  bool rospath();

protected:
private:
  // store pose frame by frame
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      global_poses_;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      local_poses_;
  // store point clouds frame by frame
  std::vector<pcl::PointCloud<Surfel>::Ptr> semantic_map_;
  pcl::PointCloud<Surfel>::Ptr active_map_;
  pcl::PointCloud<Surfel>::Ptr unactive_map_;
  int time_gap_;
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
  int current_frame_id_;
  // navigation path message
  nav_msgs::Path global_path_;
};

#endif // SRC_SMTEST_H
