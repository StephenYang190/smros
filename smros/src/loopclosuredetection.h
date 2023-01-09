//
// Created by tongda on 2022/4/26.
//

#ifndef SRC_LOOPCLOSUREDETECTION_H
#define SRC_LOOPCLOSUREDETECTION_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <iostream>
#include <mutex>
#include <queue>
#include "Scancontext/Scancontext.h"

#include "smros_msgs/Keyframe.h"
#include "surfel.h"

class LoopClosureDetection {
 public:
  LoopClosureDetection();
  void PointCloudCallback(const sensor_msgs::PointCloud2& msg);
  void OdometryCallback(const nav_msgs::Odometry& msg);

 protected:
 private:
  // ros node handle
  ros::NodeHandle nh_;
  // scan context manager
  SCManager scManager_;
  // distance threshold
  float max_distance_;
  // angle thresholdmap
  float max_angle_;
  std::queue<pcl::PointCloud<Surfel>::Ptr> tmp_point_cloud_;
  ros::Publisher keyframe_pub_;
  std::mutex tmp_point_cloud_mutex_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber odometry_sub_;
};

#endif  //SRC_LOOPCLOSUREDETECTION_H
