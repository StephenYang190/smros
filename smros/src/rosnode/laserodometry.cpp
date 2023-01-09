//
// Created by tongda on 2022/4/25.
//

#include "odometry.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry");
  Odometry odometry;
  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub =
      nh.subscribe("surfel", 1000, &Odometry::callBack, &odometry);
  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}