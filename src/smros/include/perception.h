//
// Created by tongda on 2022/4/24.
//

#ifndef SRC_PERCEPTION_H
#define SRC_PERCEPTION_H

#include "RangenetAPI.hpp"
#include <image_transport/image_transport.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "semantic_surfel.h"

class Perception {
public:
  Perception();

  // pre-process point clouds(transform to surfel based point clouds)
  void PointCloudCallback(const sensor_msgs::PointCloud2 &msg);

protected:
  // compute the matching between the Vertex map and point clouds
  bool generateVertexMap(std::vector<int> &point_labels);

  bool computeUVIndex(int point_index, int &u, int &v, float &r_xyz);

  // transform point to surfel
  void point2Surfel(int timestamp);

  // verify range image
  void publishRangeImage();

  // compute normal based on range image
  void generateNormal();

private:
  // rangenet
  std::shared_ptr<RangenetAPI> net_;
  // ros node handle
  ros::NodeHandle nh_;
  ros::Publisher surfel_pub_;
  std::string work_directory_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_point_cloud_;
  ros::Subscriber point_cloud_sub_;
  // height and width of Vertex map
  int height_, width_;
  // fov up and down for the range of interesting(ROI)
  float fov_up_, fov_down_, fov_;
  // point range constriction
  int max_distance_;
  int min_distance_;
  // point a line have
  int linenum;
  bool remove_vehicle_, downsample_;
  // Vertex map, which store the index of point in point clouds
  std::shared_ptr<VertexMap> vertex_maps_ptr_;
  // initial confidence
  float initial_confidence_;
  // pixel size
  float p_;
  image_transport::ImageTransport img_nh_;
  image_transport::Publisher range_image_pub_;
  int output_point_number_{0};
};

#endif // SRC_PERCEPTION_H
