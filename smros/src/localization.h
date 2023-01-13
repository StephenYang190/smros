//
// Created by tongdayang on 1/12/23.
//

#ifndef SRC_LOCALIZATION_H
#define SRC_LOCALIZATION_H

#include "Scancontext/Scancontext.h"
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "semanticmap.h"

class Localization {
public:
  Localization();
  ~Localization();
  void SurfelCallback(const sensor_msgs::PointCloud2 &msg);
  void publishMap(const ros::TimerEvent &event);

protected:
  pose_type computePose(pcl::PointCloud<Surfel>::Ptr input_points,
                        pcl::PointCloud<Surfel>::Ptr target_points,
                        Eigen::Matrix4f &initial_pose);
  void laserOdometry();
  bool loopDetection();

private:
  ros::NodeHandle nh_;
  pcl::PointCloud<Surfel>::Ptr current_point_cloud_;
  ros::Publisher local_pose_pub_;
  ros::Publisher global_map_pub_;
  pose_type current_pose;
  ros::Subscriber surfel_sub_;
  // scan context manager
  SCManager scManager_;
  // distance threshold
  float max_distance_;
  // angle thresholdmap
  float max_angle_;
  SemanticMap map_;
  ros::Timer timer_;
  uint32_t current_seq_;
  // timestamp
  int current_frame_id_;
};

#endif // SRC_LOCALIZATION_H
