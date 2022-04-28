//
// Created by tongda on 2022/4/25.
//

#ifndef SRC_SMTEST_H
#define SRC_SEMANTICMAP_H

#define PCL_NO_PRECOMPILE

#include <pcl_ros/point_cloud.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <queue>

#include "surfel.h"
#include "backendopt.h"
#include "smros_msgs/Keyframe.h"

class SemanticMap {
private:
    // store pose frame by frame
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> global_poses_;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> local_poses_;
    std::queue<nav_msgs::Odometry> tmp_local_poses_;
    // store point clouds frame by frame
    std::vector<pcl::PointCloud<Surfel>::Ptr> semantic_map_;
    std::queue<sensor_msgs::PointCloud2ConstPtr> tmp_point_cloud_;
    // ros nodehandle
    ros::NodeHandle nh_;
    // ros publisher
    ros::Publisher global_map_pub_;
    ros::Publisher global_path_pub_;
    // pose graph
    BackEndOpt pose_graph_;
    // information metrix
    Eigen::DiagonalMatrix<double, 6> info_;
    // parameter used to control the time to optimization
    int detect_threshold_;
    // parameter used to mark loop closure
    bool loop_closure_detection_;
    // path to save pose
    std::string pose_out_path_;
    // timestamp
    int timestamp_;
    // navigation path message
    nav_msgs::Path global_path_;
    // mutex to control multi thread
    std::mutex global_pose_mutex_;
    std::mutex tmp_point_cloud_mutex_;
    std::mutex tmp_pose_mutex_;

protected:

public:
    SemanticMap();
    // add pose to pose list
    void callBackPose(const nav_msgs::Odometry& msg);
    void callBackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
    void callBackLoopClosure(const smros_msgs::Keyframe& msg);
};


#endif //SRC_SMTEST_H
