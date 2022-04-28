//
// Created by tongda on 2022/4/24.
//

#ifndef SRC_PREPROCESS_H
#define SRC_PREPROCESS_H

#include <ros/ros.h>
#include <iostream>
#include "RangenetAPI.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "surfel.h"
#include "vertexmap.h"

class Preprocess {
private:
    // rangenet
    std::shared_ptr<RangenetAPI> net_;
    // frame to store current point clouds and tansform to surfel based point clouds
    std::shared_ptr<VertexMap> current_frame_;
    // ros node handle
    ros::NodeHandle nh_;
    ros::Publisher surfel_pub_;
    bool remove_vehicle_, downsample_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_;

protected:

public:
    Preprocess();
    // pre-process point clouds(transform to surfel based point clouds)
    void callBack(const sensor_msgs::PointCloud2& msg);
};


#endif //SRC_PREPROCESS_H
