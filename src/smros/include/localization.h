//
// Created by tongdayang on 1/12/23.
//

#ifndef SRC_LOCALIZATION_H
#define SRC_LOCALIZATION_H

#include "Scancontext/Scancontext.h"
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "pose_estimate.h"

class Localization {
public:
  Localization();

  ~Localization();

protected:
  void laserOdometry();

  void SurfelCallback(const sensor_msgs::PointCloud2 &msg);

private:
  ros::NodeHandle nh_;
  pcl::PointCloud<SemanticSurfel>::Ptr current_point_cloud_;
  pcl::PointCloud<SemanticSurfel>::Ptr last_point_cloud_;
  ros::Publisher local_pose_pub_;
  Eigen::Matrix4d current_pose;
  ros::Subscriber surfel_sub_;
  // distance threshold
  float max_distance_;
  // angle thresholdmap
  float max_angle_;
  uint32_t current_seq_;
  // nonlinear estimate
  std::unique_ptr<NonlinearEstimate> odometry;
  int width_{3600};
  int height_{64};
  float fov_{27.0};
  float fov_up_{2.0};
};

#endif // SRC_LOCALIZATION_H
