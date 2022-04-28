//
// Created by tongda on 2022/4/28.
//

#include "testgtsam.h"

testgtsam::testgtsam()
{
    // initial vector
    global_poses_.resize(0);
    local_poses_.resize(0);
    semantic_map_.resize(0);

    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map", 1000);
    global_path_pub_ = nh_.advertise<nav_msgs::Path>("gloabl_path", 1000);

    auto& diag = info_.diagonal();
    // translational noise is smaller than ro tational noise.
    double transNoise = 1.0;
    double rotNoise = 1.0;
    diag[0] = (transNoise * transNoise);
    diag[1] = (transNoise * transNoise);
    diag[2] = (transNoise * transNoise);

    diag[3] = (rotNoise * rotNoise);
    diag[4] = (rotNoise * rotNoise);
    diag[5] = (rotNoise * rotNoise);

    nh_.getParam("detection_treshold", detect_threshold_);
    nh_.getParam("is_loop_detection", loop_closure_detection_);
    nh_.getParam("pose_out_path", pose_out_path_);

    timestamp_ = 0;
}

void testgtsam::callBackPose(const nav_msgs::Odometry &msg) {
    tmp_pose_mutex_.lock();
    tmp_local_poses_.push(msg);
    tmp_pose_mutex_.unlock();
}

void testgtsam::callBackPointCloud(const sensor_msgs::PointCloud2& msg) {
    pcl::PointCloud<Surfel>::Ptr point_cloud(new pcl::PointCloud<Surfel>);
    pcl::fromROSMsg(msg, *point_cloud);
    tmp_point_cloud_mutex_.lock();
    tmp_point_cloud_.push(point_cloud);
    tmp_point_cloud_mutex_.unlock();
}

void testgtsam::callBackLoopClosure(const smros_msgs::Keyframe& msg) {
    int pre_index = msg.pre_id;
    int crt_index = msg.crt_id;
    std::cout << "crt: " << crt_index << " timestamp: " << timestamp_ << std::endl;
    // get point cloud
    pcl::PointCloud<Surfel>::Ptr pointcloud;
    tmp_point_cloud_mutex_.lock();
    while(!tmp_point_cloud_.empty())
    {
        pointcloud = tmp_point_cloud_.front();
        if(pointcloud->header.seq == crt_index)
        {
            tmp_point_cloud_.pop();
            break;
        }
        else if(pointcloud->header.seq < crt_index)
        {
            tmp_point_cloud_.pop();
        }
        else
        {
            break;
        }
    }
    tmp_point_cloud_mutex_.unlock();
    if(pointcloud && pointcloud->header.seq == crt_index)
    {
        semantic_map_.push_back(pointcloud);
    }
    // get pose
    nav_msgs::Odometry odometry;
    tmp_pose_mutex_.lock();
    while(!tmp_local_poses_.empty())
    {
        odometry = tmp_local_poses_.front();
        if(odometry.header.seq == crt_index)
        {
            tmp_local_poses_.pop();
            break;
        }
        else if(odometry.header.seq < crt_index)
        {
            tmp_local_poses_.pop();
        }
        else
        {
            break;
        }
    }
    tmp_pose_mutex_.unlock();
    if(odometry.header.seq == crt_index)
    {
        Eigen::Matrix4f local_pose = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f global_pose = Eigen::Matrix4f::Identity();
        // get rotation matrix
        Eigen::Quaternionf q(odometry.pose.pose.orientation.w,
                             odometry.pose.pose.orientation.x,
                             odometry.pose.pose.orientation.y,
                             odometry.pose.pose.orientation.z);
        q.normalize();
        local_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        // get t matrix
        local_pose(0, 3) = odometry.pose.pose.position.x;
        local_pose(1, 3) = odometry.pose.pose.position.y;
        local_pose(2, 3) = odometry.pose.pose.position.z;
        // compute global pose
        if(timestamp_ > 0)
        {
            global_pose = global_poses_[timestamp_ - 1] * local_pose;
        }
        // svae global pose
        global_pose_mutex_.lock();
        global_poses_.push_back(global_pose);
        global_pose_mutex_.unlock();
        // set optional graph node and edge
        pose_graph_.setInitialValues(timestamp_, global_pose.cast<double>());
        if(timestamp_ > 0)
        {
            pose_graph_.addEdge(0, 1, Eigen::Matrix4d::Identity());
        }
        timestamp_++;
    }
    // do not have loop closure
}

testgtsam::~testgtsam() {

}
