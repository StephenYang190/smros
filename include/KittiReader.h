//
// Created by tongda on 2022/4/24.
//

#ifndef SRC_KITTIREADER_H
#define SRC_KITTIREADER_H

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>

class KittiReader {
private:
    // ros node handle
    ros::NodeHandle nh_;
    ros::Publisher point_cloud_pub_;
    std::string pcd_in_path_;
    std::string sequence_;
    std::vector<std::string> pcd_lists_;
    int current_index_;

protected:

public:
    KittiReader();
    bool readPointCloud();
};


#endif //SRC_KITTIREADER_H
