//
// Created by tongda on 2021/12/16.
//

#ifndef SMROS_SUMASLAM_H
#define SMROS_SUMASLAM_H


#include "RangenetAPI.hpp"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rv/ParameterList.h"
#include "Frame.h"
#include "SurfelMap.h"


class SumaSLAM {
private:
    std::shared_ptr<RangenetAPI> net_;
    std::shared_ptr<Frame> current_frame_;
    std::shared_ptr<Frame> last_frame_;
    rv::ParameterList params_;
    uint32_t timestamp_;
    pcl::PointCloud<pcl::PointXYZRGB> odometry_result_;
    std::shared_ptr<SurfelMap> map_;

public:
    SumaSLAM(std::string parameter_path = "");
    ~SumaSLAM();
    bool step(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi);
    bool render();
    bool preprocess(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi);
    bool odometry();
    bool initialSystem(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi);
    bool mapUpdate();


};


#endif //SMROS_SUMASLAM_H
