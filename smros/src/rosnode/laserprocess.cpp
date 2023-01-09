//
// Created by tongda on 2022/4/24.
//

#include "../perception.h"
int main(int argc, char** argv) {
  ros::init(argc, argv, "pre_process");
  Preprocess preprocess;
  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub =
      nh.subscribe("point_cloud_in", 1000, &Preprocess::callBack, &preprocess);
  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}