//
// Created by tongda on 2021/12/16.
//

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "point2surfel.h"


ros::Publisher pub;
Point2Surfel point_2_surfel;

// Callbcak function
void kittiPointCloudReceive(const sensor_msgs::PointCloud2ConstPtr &pointCloud2)
{
//    ROS_INFO_STREAM("row step: " << pointCloud2->row_step);
//    ROS_INFO_STREAM("width: " << pointCloud2->width);
//    ROS_INFO_STREAM("height: " << pointCloud2->height);

    auto pointcloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pointCloud2, *pointcloud_xyzi);

    // processing
    ROS_INFO_STREAM("Begin.");
    ROS_INFO_STREAM("Input Point numbers: " << pointcloud_xyzi->points.size());
    auto pointcloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pointcloud_xyzi, *pointcloud_rgb);

    point_2_surfel.transform(*pointcloud_xyzi, *pointcloud_rgb);
    ROS_INFO_STREAM("Output Point numbers: " << pointcloud_rgb->points.size());
    ROS_INFO_STREAM("End1.");

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*pointcloud_rgb, msg);
    pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "semantic_mapping");
    ros::NodeHandle nh;
    // pub difine
    pub = nh.advertise<sensor_msgs::PointCloud2> ("SemanticMap", 1000);

    // sub difine
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, kittiPointCloudReceive);

    ros::spin();
}