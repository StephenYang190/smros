//
// Created by tongda on 2021/12/17.
//

#ifndef SMROS_FRAME_H
#define SMROS_FRAME_H


#include <pcl_conversions/pcl_conversions.h>
#include <rv/ParameterList.h>
#include <memory>
#include <Eigen/Dense>
#include "surfel.h"
#include <pcl/filters/filter.h>


namespace frm{
    struct map{
        Eigen::Vector3d vertex_map;
        Eigen::Vector3d normal_map;
        int index;
        float semantic_map;
        float radius;
    };
}

class Frame {
private:
    int height_, width_;
    float fov_up_, fov_down_, fov_;
    pcl::PointCloud<Surfel> pointcloud_;
    std::vector<std::vector<frm::map>> maps_;
    std::vector<int> labels_;
    std::vector<float> label_probability_;
    float initial_confidence_;
    float p_;

public:
    Frame(rv::ParameterList parameter_list, float init_confidence);
    ~Frame();
    pcl::PointCloud<Surfel>& setPointCloud();
    bool generateMap(int timestamp);
    std::vector<int> & setLabels() {return labels_;}
    std::vector<float> & setLabelProbability() {return label_probability_;}
    const pcl::PointCloud<Surfel>& getPointClouds() {return pointcloud_;}
    pcl::PointCloud<Surfel>::Ptr getPointCloudsPtr() {return pointcloud_.makeShared();}
    const std::vector<std::vector<frm::map>> & getMaps() const {return maps_;};
    bool clearIndexMap();
};


#endif //SMROS_FRAME_H
