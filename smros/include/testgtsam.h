//
// Created by tongda on 2022/4/28.
//

#ifndef SRC_TESTGTSAM_H
#define SRC_TESTGTSAM_H

#include <pcl_ros/point_cloud.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <queue>
#include "smros_msgs/Keyframe.h"

#include "surfel.h"
#include "backendopt.h"

class testgtsam {
private:
    // store pose frame by frame
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> global_poses_;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> local_poses_;
    std::queue<nav_msgs::Odometry> tmp_local_poses_;
    // store point clouds frame by frame
    std::vector<pcl::PointCloud<Surfel>::Ptr> semantic_map_;
    std::queue<pcl::PointCloud<Surfel>::Ptr> tmp_point_cloud_;
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
    testgtsam();
    ~testgtsam();
    // add pose to pose list
    void callBackPose(const nav_msgs::Odometry& msg);
    void callBackPointCloud(const sensor_msgs::PointCloud2& msg);
    void callBackLoopClosure(const smros_msgs::Keyframe& msg);
};


#endif //SRC_TESTGTSAM_H
