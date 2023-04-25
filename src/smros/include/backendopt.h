/* BackEndOpt class
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

class BackEndOpt {
private:
    gtsam::NonlinearFactorGraph pose_graph_;
    gtsam::Values initials_;
    gtsam::Values results_;
    Eigen::DiagonalMatrix<double, 6> info_;

public:
    BackEndOpt();

    ~BackEndOpt();

    bool setInitialValues(int id, const Eigen::Matrix4d &initial_estimate);

    bool addEdge(int from, int to, const Eigen::Matrix4d &measurement,
                 const Eigen::Matrix<double, 6, 6> &information);

    bool addEdge(int from, int to, const Eigen::Matrix4d &measurement);

    bool optimize(int num_iters);

    bool updatePoses(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &pose);
};


#endif //SRC_BACKENDOPT_H
