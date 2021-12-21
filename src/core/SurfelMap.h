//
// Created by tongda on 2021/12/19.
//

#ifndef SRC_SURFELMAP_H
#define SRC_SURFELMAP_H

#include "surfel.h"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include "Frame.h"


class SurfelMap {
private:
    std::vector<Eigen::Matrix4f> poses_;
    pcl::PointCloud<Surfel> surfels_;
    float width_, height_;
    float p_stable_, p_prior_, odds_p_prior_;
    float sigma_angle_2_, sigma_distance_2_;
    float initial_confidence_;

public:
    SurfelMap(rv::ParameterList parameter_list);
    ~SurfelMap();
    bool pushBackPose(Eigen::Matrix4f pose);
    Eigen::Matrix4f getPose(int timestamp);
    bool updateMap(const Frame & current_frame, int timestamp);
    bool mapInitial(const Frame & current_frame);
    float updateConfidence(float confidence, float angle_2, float distance_2);
};


#endif //SRC_SURFELMAP_H
