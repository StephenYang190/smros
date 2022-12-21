//
// Created by tongda on 2022/4/26.
//

#include "LoopClosureDetection.h"

LoopClosureDetection::LoopClosureDetection() {
    nh_.getParam("max_distance_gap", max_distance_);
    nh_.getParam("max_angle_gap", max_angle_);
    keyframe_pub_ = nh_.advertise<smros_msgs::Keyframe>("keyframe_id", 1000);
}

void LoopClosureDetection::callBackPointCloud(const sensor_msgs::PointCloud2 &msg) {
    pcl::PointCloud<Surfel>::Ptr pointcloud(new pcl::PointCloud<Surfel>);
    pcl::fromROSMsg(msg, *pointcloud);
    tmp_point_cloud_mutex_.lock();
    tmp_point_cloud_.push(pointcloud);
    tmp_point_cloud_mutex_.unlock();
}

void LoopClosureDetection::callBackOdometry(const nav_msgs::Odometry &msg) {
    int id = msg.header.seq;
    tmp_point_cloud_mutex_.lock();
    pcl::PointCloud<Surfel>::Ptr pointcloud;
    while(!tmp_point_cloud_.empty())
    {
        pointcloud = tmp_point_cloud_.front();
        if(pointcloud->header.seq < id)
        {
            tmp_point_cloud_.pop();
        }
        else if(pointcloud->header.seq == id)
        {
            tmp_point_cloud_.pop();
            break;
        }
        else
        {
            break;
        }
    }
    tmp_point_cloud_mutex_.unlock();
    if(pointcloud && pointcloud->header.seq == id)
    {
        bool od_keyframe = true;
        bool sc_keyframe = true;
//        // compute distance between two frames
//        Eigen::Vector3f t(msg.pose.pose.position.x,
//                          msg.pose.pose.position.y,
//                          msg.pose.pose.position.z);
//        float distance = t.squaredNorm();
//        // compute angle change
//        Eigen::Quaternionf q(msg.pose.pose.orientation.w,
//                             msg.pose.pose.orientation.x,
//                             msg.pose.pose.orientation.y,
//                             msg.pose.pose.orientation.z);
//        q.normalize();
//        float angle_change = q.squaredNorm();
//        if(distance < max_distance_ && angle_change < max_angle_)
//        {
//            od_keyframe = true;
//        }
        // find loop closure with scan context
        scManager_.makeAndSaveScancontextAndKeys(*pointcloud);
        auto pair_result = scManager_.detectLoopClosureID();
        int pre_index = pair_result.first;
        if( pre_index == -1 || pre_index > id) {
            if(!od_keyframe)
            {
                scManager_.popBack();
            }
            sc_keyframe = false;
        }
        if(od_keyframe || sc_keyframe)
        {
            smros_msgs::Keyframe keyframe_msg;
            keyframe_msg.pre_id = pre_index;
            keyframe_msg.crt_id = id;
            keyframe_pub_.publish(keyframe_msg);
        }
    }
    else
    {
        return ;
    }
}
