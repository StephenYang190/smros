//
// Created by tongda on 2022/4/24.
//

#ifndef SRC_PERCEPTION_H
#define SRC_PERCEPTION_H

#include "RangenetAPI.hpp"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "surfel.h"
#include "vertexmap.h"

class Perception {
public:
  Perception();
  // pre-process point clouds(transform to surfel based point clouds)
  void PointCloudCallback(const sensor_msgs::PointCloud2 &msg);

protected:
private:
  // rangenet
  std::shared_ptr<RangenetAPI> net_;
  // frame to store current point clouds and tansform to surfel based point
  // clouds
  std::shared_ptr<VertexMap> current_frame_;
  // ros node handle
  ros::NodeHandle nh_;
  ros::Publisher surfel_pub_;
  std::string work_directory_;
  bool remove_vehicle_, downsample_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_;
  ros::Subscriber point_cloud_sub_;
};

#endif // SRC_PERCEPTION_H
