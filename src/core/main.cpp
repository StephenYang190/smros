//
// Created by tongda on 2021/12/16.
//

#include "sumaslam.h"

// Callbcak function
void kittiPointCloudReceive(const sensor_msgs::PointCloud2ConstPtr &pointCloud2)
{
//    ROS_INFO_STREAM("row step: " << pointCloud2->row_step);
//    ROS_INFO_STREAM("width: " << pointCloud2->width);
//    ROS_INFO_STREAM("height: " << pointCloud2->height);

//    auto pointcloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::fromROSMsg(*pointCloud2, *pointcloud_xyzi);

    // processing
//    ROS_INFO_STREAM("Begin.");
//    ROS_INFO_STREAM("Input Point numbers: " << pointcloud_xyzi->points.size());
//    sumaslam.step(*pointcloud_xyzi);

//    auto pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    sumaslam.generateMap(*pointcloud);
//    std::cout << "number of points in active map: " << pointcloud->size() << std::endl;

//    ROS_INFO_STREAM("Output Point numbers: " << pointcloud_rgb->points.size());
//    ROS_INFO_STREAM("End1.");

//    sensor_msgs::PointCloud2 msg;
//    pcl::toROSMsg(*pointcloud, msg);
//    pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "semantic_mapping");
    SumaSLAM sumaslam;
    sumaslam.init();
    if(sumaslam.readFromFile())
    {
        std::cout << "SemanticMap construction well." << std::endl;
    }
    ros::Rate r(10);
    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

//    sumaslam.testLoopsure();

//    ros::Rate rate(2);
//    ros::AsyncSpinner spinner(1);
//    spinner.start();
//    // pub difine
//    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("SemanticMap", 1000);
//
//    // sub difine
//    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, kittiPointCloudReceive);
//
//    while(ros::ok())
//    {
//        auto pointcloud(new pcl::PointCloud<Surfel>);
//        if(!sumaslam.generateMap(*pointcloud))
//        {
//            rate.sleep();
//            continue;
//        }
//        sensor_msgs::PointCloud2 msg;
//        pcl::toROSMsg(*pointcloud, msg);
//        msg.header.frame_id = "velodyne";
//        pub.publish(msg);
//        rate.sleep();
//    }

    return 0;
}