//
// Created by tongdayang on 1/12/23.
//

#include "localization.h"

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include "utils.h"

const float PI = acos(-1);

Localization::Localization() {
    nh_.getParam("max_distance_gap", max_distance_);
    nh_.getParam("max_angle_gap", max_angle_);
    local_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("local_pose", 1000);
    current_pose = Eigen::Matrix4d::Identity();
    surfel_sub_ =
            nh_.subscribe("surfel", 1000, &Localization::SurfelCallback, this);

    // set last point cloud to empty
    last_point_cloud_.reset();
    // init odometry
    odometry.SetMaxIterations(10);
    odometry.SetResolution(0.05);
}

Localization::~Localization() {}

void Localization::SurfelCallback(const sensor_msgs::PointCloud2 &msg) {
    // accept new point cloud
    current_point_cloud_.reset(new pcl::PointCloud<SemanticSurfel>);
    pcl::fromROSMsg(msg, *current_point_cloud_);
    current_seq_ = msg.header.seq;
    laserOdometry();
    // set last point cloud
    last_point_cloud_ = current_point_cloud_;
    odometry.SetTargetPointCloud(last_point_cloud_);
}

void Localization::laserOdometry() {
    if (last_point_cloud_) {
        odometry.SetSourcePointCloud(current_point_cloud_);
        odometry.Align();
        current_pose = odometry.GetFinalResult();
//    Eigen::Matrix3d rotation = current_pose.block<3, 3>(0, 0);
//    Eigen::Vector3d t(current_pose.block<3, 1>(0, 3));
//    float distance = t.norm();
//    float yaw_angle = rotation.eulerAngles(2, 1, 0)[0] / PI * 180;
        //    std::cout << "yaw angle : " << yaw_angle << std::endl;
        //    std::cout << "distance between two frames is : " << distance <<
        //    std::endl;
    }
    // transfer matrix to quaternion
    auto q = odometry.GetQuaternion();
    auto t = odometry.GetTranslation();
    // publish pose
    nav_msgs::Odometry current_odometry;
    current_odometry.header.frame_id = "velodyne";
    current_odometry.child_frame_id = "laser_odom";
    current_odometry.header.seq = current_seq_;
    current_odometry.pose.pose.orientation.x = q[0];
    current_odometry.pose.pose.orientation.y = q[1];
    current_odometry.pose.pose.orientation.z = q[2];
    current_odometry.pose.pose.orientation.w = q[3];
    current_odometry.pose.pose.position.x = t[0];
    current_odometry.pose.pose.position.y = t[1];
    current_odometry.pose.pose.position.z = t[2];
    local_pose_pub_.publish(current_odometry);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization");
    Localization localization;
    ros::spin();
    return 0;
}