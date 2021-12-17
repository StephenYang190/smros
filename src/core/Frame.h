//
// Created by tongda on 2021/12/17.
//

#ifndef SMROS_FRAME_H
#define SMROS_FRAME_H


#include <pcl_conversions/pcl_conversions.h>
#include <rv/ParameterList.h>
#include <memory>
#include <Eigen/Dense>


namespace frm{
    struct map{
        Eigen::Vector3d vertex_map;
        Eigen::Vector3d normal_map;
        float semantic_map;
    };
}

class Frame {
private:
    float height_, width_;
    float fov_up_, fov_down_, fov_;
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_;
    std::vector<std::vector<frm::map>> maps_;
    std::vector<int> labels_;
    std::vector<float> label_probability_;

public:
    Frame(rv::ParameterList parameter_list);
    ~Frame();
    pcl::PointCloud<pcl::PointXYZRGB>& setPointCloud();
    bool generateMap();
    std::vector<int> & setLabels() {return labels_;}
    std::vector<float> & setLabelProbability() {return label_probability_;}

};


#endif //SMROS_FRAME_H
