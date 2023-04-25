//
// Created by tongda on 2022/4/25.
//

#ifndef SRC_SEMANTICMAP_H
#define SRC_SEMANTICMAP_H

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_ros/point_cloud.h>
#include <queue>

#include "Scancontext/Scancontext.h"
#include "backendopt.h"
#include "semantic_surfel.h"

using PoseType = Eigen::Matrix4d;

class SemanticMap {
public:
  SemanticMap();

  ~SemanticMap();

protected:
  void SurfelCallback(const sensor_msgs::PointCloud2 &msg);

  void PoseCallback(const nav_msgs::Odometry &msg);

  void PublishMap(const ros::TimerEvent &event);

  bool LoopDetection();

  // update map
  bool UpdateMap(PoseType crt_pose = PoseType::Identity());

  // generate global map
  bool GenerateMap(pcl::PointCloud<SemanticSurfel>::Ptr global_map);

  void SaveLocalPose(PoseType crt_pose);

  pcl::PointCloud<SemanticSurfel>::Ptr GetUnActiveMapPtr(int id);

  // set loop edge in factor graph
  bool SetLoopSureEdge(int from, int to, PoseType &pose);

private:
  // store pose frame by frame
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
      global_poses_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
      local_poses_;
  // store point clouds frame by frame
  std::vector<pcl::PointCloud<SemanticSurfel>::Ptr> semantic_map_;
  pcl::PointCloud<SemanticSurfel>::Ptr unactive_map_;
  int time_gap_;
  // ros nodehandle
  ros::NodeHandle nh_;
  std::string work_directory_;
  ros::Subscriber surfel_sub_;
  ros::Subscriber pose_sub_;
  ros::Timer timer_;
  // ros publisher
  ros::Publisher global_map_pub_;
  // pose graph
  BackEndOpt factor_graph_;
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
  int last_loop_index_;
  // scan context manager
  SCManager scManager_;
  std::queue<sensor_msgs::PointCloud2> point_cloud_queue_;
  pcl::PointCloud<SemanticSurfel>::Ptr current_point_cloud_;
};

#endif // SRC_SEMANTICMAP_H
