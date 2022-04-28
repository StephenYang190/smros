//
// Created by tongda on 2022/4/26.
//

#ifndef SRC_LOOPCLOSUREDETECTION_H
#define SRC_LOOPCLOSUREDETECTION_H


#include <ros/ros.h>
#include <iostream>
#include <Scancontext/Scancontext.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <mutex>

#include "surfel.h"
#include "smros_msgs/Keyframe.h"


class LoopClosureDetection {
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
protected:

public:
    LoopClosureDetection();
    void callBackPointCloud(const sensor_msgs::PointCloud2& msg);
    void callBackOdometry(const nav_msgs::Odometry& msg);
};


#endif //SRC_LOOPCLOSUREDETECTION_H
