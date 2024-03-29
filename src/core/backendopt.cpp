//
// Created by tongda on 2022/1/5.
//

#include "backendopt.h"

using namespace gtsam;

BackEndOpt::BackEndOpt() :
    pose_graph_(),
    initials_(),
    results_()
{
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
}

bool BackEndOpt::addEdge(int from, int to, const Eigen::Matrix4d &measurement,
                         const Eigen::Matrix<double, 6, 6>& information) {
    std::cout << information(0, 1) << std::endl;
    auto odometry_model = noiseModel::Gaussian::Information(information);

    NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(from, to, Pose3(measurement), odometry_model));
    pose_graph_.add(factor);
    return false;
}

bool BackEndOpt::addEdge(int from, int to, const Eigen::Matrix4d &measurement) {
    auto odometry_model = noiseModel::Gaussian::Information(info_);

    NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(from, to, Pose3(measurement), odometry_model));
    pose_graph_.add(factor);
    return false;
}

bool BackEndOpt::setInitialValues(int id, const Eigen::Matrix4d& initial_estimate) {
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

bool BackEndOpt::optimize(int num_iters) {
    LevenbergMarquardtParams params;
    params.maxIterations = num_iters;
    params.setVerbosity("TERMINATION");  // this will show info about stopping conditions
    params.setVerbosity("SILENT");
    LevenbergMarquardtOptimizer optimizer(pose_graph_, initials_, params);

    results_ = optimizer.optimize();
    return true;
}

bool BackEndOpt::updatePoses(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &pose) {
    for (const auto& pair : results_) {
        if (pair.key >= pose.size()) continue;

        pose[pair.key] = pair.value.cast<Pose3>().matrix().cast<float>();
    }

    return false;
}

BackEndOpt::~BackEndOpt() {

}
