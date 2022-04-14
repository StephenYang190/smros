/* Optimization class
 * Create by Tongda Yang
 * This class is used to implement pose graph optimization with gtsam
 * */

#ifndef SRC_BACKENDOPT_H
#define SRC_BACKENDOPT_H

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Scancontext/Scancontext.h>

class Optimization {
private:
    gtsam::NonlinearFactorGraph pose_graph_;
    gtsam::Values initials_;
    gtsam::Values results_;

public:
    Optimization();
    ~Optimization();
    bool setInitialValues(int id, const Eigen::Matrix4d& initial_estimate);
    bool addEdge(int from, int to, const Eigen::Matrix4d & measurement,
                 const Eigen::Matrix<double, 6, 6>& information);
    bool optimize(int num_iters);
    bool updatePoses(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> & pose);
};

/* Bcak End Class
 * Create by Tongda Yang
 * Include scancontext and odometry loopsure detection
 * Include optimization
 * */
class BackEndOpt {
private:
    // scan context manager
    SCManager scManager_;
    // optimization
    Optimization pose_graph_;
    // store position of each frame
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> position_s_;
    // inactive map time
    int inactive_;
    // oodometry max distance between two frames
    float max_dis_;
    // ros nodehanle
    ros::NodeHandle nh_;
protected:
    // find loop closure from scan context
    std::pair<int, int> scDetection(std::shared_ptr<pcl::PointCloud<Surfel>> pointcloud);
    // find loop closure from odometry
    std::pair<int, int> odDetection(Eigen::Matrix4f& pose);
public:
    BackEndOpt();
    ~BackEndOpt() = default;
    std::pair<int, int> detectLoopClosure(std::shared_ptr<pcl::PointCloud<Surfel>> pointcloud, Eigen::Matrix4f& pose);
    bool optimise(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> & pose_s);
    bool setInitialValues(int id, const Eigen::Matrix4d& initial_estimate);
    bool addEdge(int from, int to, const Eigen::Matrix4d & measurement,
                 const Eigen::Matrix<double, 6, 6>& information);
};

#endif //SRC_BACKENDOPT_H
