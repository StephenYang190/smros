//
// Created by tongda on 2022/4/25.
//

#include "Odometry.h"

Eigen::Matrix4f computePose(pcl::PointCloud<Surfel>::Ptr input_points,
                      pcl::PointCloud<Surfel>::Ptr target_points,
                      Eigen::Matrix4f& initial_pose) {
    pcl::PointCloud<Surfel> odometry_result;
    pclomp::NormalDistributionsTransform<Surfel, Surfel> ndt;
    ndt.setNumThreads(4);
    // Setting point cloud to be aligned.
    ndt.setInputSource (input_points);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (target_points);

    // Calculating required rigid transform to align the input cloud to the target cloud.
    ndt.align (odometry_result, initial_pose);

    return ndt.getFinalTransformation();
}

Odometry::Odometry() :
    last_point_cloud_(new pcl::PointCloud<Surfel>),
    current_point_cloud_(new pcl::PointCloud<Surfel>)
{
    local_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("local_pose", 1000);
    last_pose_ = Eigen::Matrix4f::Identity();
    current_pose = Eigen::Matrix4f::Identity();
    init_ = false;
}

void Odometry::callBack(const sensor_msgs::PointCloud2& msg) {
    pcl::fromROSMsg(msg, *current_point_cloud_);
    if (init_)
    {
        current_pose = computePose(last_point_cloud_, current_point_cloud_, last_pose_);
        float distance = 0;
        for(int i = 0; i < 3; i++)
        {
            float dis = current_pose(i, 3);
            distance += dis * dis;
        }
        distance = sqrt(distance);
        Eigen::Matrix3f rotation = current_pose.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation);
        std::cout << "x angle : " << q.x() << std::endl;
        std::cout << "y angle : " << q.y() << std::endl;
        std::cout << "z angle : " << q.z() << std::endl;
        std::cout << "distance between two frames is : " << distance << std::endl;
    }
    else
    {
        init_ = true;
    }
    // transfer matrix to quaternion
    Eigen::Quaternionf q(current_pose.block<3, 3>(0, 0));
    Eigen::Vector3f t;
    t << current_pose(0, 3), current_pose(1, 3), current_pose(2, 3);
    // publish pose
    nav_msgs::Odometry current_odometry;
    current_odometry.header.frame_id = "velodyne";
    current_odometry.child_frame_id = "laser_odom";
    current_odometry.header.seq = msg.header.seq;
    current_odometry.pose.pose.orientation.x = q.x();
    current_odometry.pose.pose.orientation.y = q.y();
    current_odometry.pose.pose.orientation.z = q.z();
    current_odometry.pose.pose.orientation.w = q.w();
    current_odometry.pose.pose.position.x = t.x();
    current_odometry.pose.pose.position.y = t.y();
    current_odometry.pose.pose.position.z = t.z();
    local_pose_pub_.publish(current_odometry);
    // update
    *last_point_cloud_ = *current_point_cloud_;
    last_pose_ = current_pose;
}


