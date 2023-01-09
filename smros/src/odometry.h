//
// Created by tongda on 2022/4/25.
//

#ifndef SRC_ODOMETRY_H
#define SRC_ODOMETRY_H

#define PCL_NO_PRECOMPILE

#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include "pclomp/ndt_omp.h"
#include "pclomp/ndt_omp_impl.hpp"
#include "pclomp/voxel_grid_covariance_omp_impl.hpp"

#include "surfel.h"

class Odometry {
public:
    Odometry();
    void PointCloudCallback(const sensor_msgs::PointCloud2& msg);

 protected:

private:
    ros::NodeHandle nh_;
    pcl::PointCloud<Surfel>::Ptr last_point_cloud_;
    pcl::PointCloud<Surfel>::Ptr current_point_cloud_;
    ros::Publisher local_pose_pub_;
    Eigen::Matrix4f last_pose_;
    Eigen::Matrix4f current_pose;
    bool init_;
    ros::Subscriber point_cloud_sub_;
};

Eigen::Matrix4f computePose(pcl::PointCloud<Surfel>::Ptr input_points,
                            pcl::PointCloud<Surfel>::Ptr target_points,
                            Eigen::Matrix4f& initial_pose);

#endif  //SRC_ODOMETRY_H
