//
// Created by tongda on 2022/4/25.
//

#include "../semanticmap.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "semantic_map");
  SemanticMap semanticMap;
  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub = nh.subscribe(
      "surfel", 1000, &SemanticMap::PointCloudCallback, &semanticMap);
  ros::Subscriber odometry_sub = nh.subscribe(
      "local_pose", 1000, &SemanticMap::OdometryCallback, &semanticMap);
  ros::Subscriber keyframe_sub = nh.subscribe(
      "keyframe_id", 1000, &SemanticMap::KeyframeCallback, &semanticMap);
  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}