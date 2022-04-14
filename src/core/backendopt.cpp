//
// Created by tongda on 2022/1/5.
//

#include "backendopt.h"

using namespace gtsam;

Optimization::Optimization() :
    pose_graph_(),
    initials_(),
    results_()
{

}

Optimization::~Optimization() {

}

bool Optimization::addEdge(int from, int to, const Eigen::Matrix4d &measurement,
                           const Eigen::Matrix<double, 6, 6>& information) {
    auto odometry_model = noiseModel::Gaussian::Information(information);

    NonlinearFactor::shared_ptr  factor(new BetweenFactor<Pose3>(from, to, Pose3(measurement), odometry_model));
    pose_graph_.add(factor);
    return false;
}

bool Optimization::setInitialValues(int id, const Eigen::Matrix4d& initial_estimate) {
    bool addPrior = initials_.empty();

    if (!initials_.exists(id))
        initials_.insert(id, Pose3(initial_estimate));
    else
        initials_.update(id, Pose3(initial_estimate));

    // also update intermediate result.
    if (results_.exists(id))
        results_.update(id, Pose3(initial_estimate));
    else
        results_.insert(id, Pose3(initial_estimate));

    if (addPrior) {
        Vector6 diagonal;
        diagonal << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
        noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances(diagonal);
        pose_graph_.add(PriorFactor<Pose3>(id, Pose3(), priorModel));
    }
    return false;
}

bool Optimization::optimize(int num_iters) {
    LevenbergMarquardtParams params;
    params.maxIterations = num_iters;
    params.setVerbosity("TERMINATION");  // this will show info about stopping conditions
    params.setVerbosity("SILENT");
    LevenbergMarquardtOptimizer optimizer(pose_graph_, initials_, params);

    results_ = optimizer.optimize();
    return true;
}

bool Optimization::updatePoses(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &pose) {
    for (const auto& pair : results_) {
        if (pair.key >= pose.size()) continue;

        pose[pair.key] = pair.value.cast<Pose3>().matrix().cast<float>();
    }

    return false;
}

BackEndOpt::BackEndOpt() {
    nh_.getParam("inactive_time",inactive_);
    nh_.getParam("od_dis",max_dis_);
}

std::pair<int, int> BackEndOpt::detectLoopClosure(std::shared_ptr<pcl::PointCloud<Surfel>> pointcloud, Eigen::Matrix4f& pose) {
    // scan context loop closure detection
    auto scpair = scDetection(pointcloud);
    // odometry loop closure detection
    auto odpair = odDetection(pose);
    // have sc result
    if(scpair.first != -1) return scpair;
    // have od result
    if(odpair.first != -1) return odpair;

    return scpair;
}

std::pair<int, int> BackEndOpt::scDetection(std::shared_ptr<pcl::PointCloud<Surfel>> pointcloud)
{
    std::pair<int, int> result;
    scManager_.makeAndSaveScancontextAndKeys(*pointcloud);
    auto pair_result = scManager_.detectLoopClosureID();
    result.first = pair_result.first;
    result.second = position_s_.size();

    return result;
}

std::pair<int, int> BackEndOpt::odDetection(Eigen::Matrix4f& pose)
{
    std::pair<int, int> result;
    Eigen::Vector3f now_position;
    now_position << pose(0,3), pose(1,3), pose(2,3);

    int min_distance = 0;
    int min_index = -1;
    for(int i = 0; i < position_s_.size() - inactive_; i++)
    {
        Eigen::Vector3f dis = now_position - position_s_[i];
        float distance = dis.norm();
        if(distance > max_dis_) continue;
        if(min_index == -1 || min_distance > distance)
        {
            min_index = i;
            min_distance = distance;
        }
    }

    result.first = min_index;
    result.second = position_s_.size();

    position_s_.push_back(now_position);

    return result;
}

bool BackEndOpt::setInitialValues(int id, const Eigen::Matrix4d &initial_estimate) {
    pose_graph_.setInitialValues(id, initial_estimate);
    return true;
}

bool BackEndOpt::addEdge(int from, int to,
                         const Eigen::Matrix4d &measurement,
                         const Eigen::Matrix<double, 6, 6> &information) {
    pose_graph_.addEdge(from, to, measurement, information);
    return true;
}
