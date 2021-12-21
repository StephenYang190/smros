//
// Created by tongda on 2021/12/16.
//

#include "SumaSLAM.h"
#include <pcl/registration/icp.h>

SumaSLAM::SumaSLAM(std::string parameter_path)
{
    parseXmlFile(parameter_path, params_);

    std::cout << "Init rangenet." << std::endl;
    net_ = std::make_shared<RangenetAPI>(params_);

    current_frame_ = std::make_shared<Frame>(params_);

    timestamp_ = 0;
}

bool SumaSLAM::preprocess(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi)
{
    std::cout << "Transform data." << std::endl;
    int num_points = point_clouds_xyzi.points.size();
    // store pointcloud in vector type
    std::vector<float> points_xyzi_list(num_points * 4);

    // frame parameter reference
    pcl::PointCloud<pcl::PointXYZRGB>& point_clouds_rgb = current_frame_->setPointCloud();
    std::vector<int>& labels = current_frame_->setLabels();
    std::vector<float>& labels_prob = current_frame_->setLabelProbability();
    // resize
    pcl::copyPointCloud(point_clouds_xyzi, point_clouds_rgb);
    labels.resize(num_points);
    labels_prob.resize(num_points);
    // transform pointcloud to vector
    for(int i = 0; i < num_points; i++)
    {
        int iter = i * 4;
        points_xyzi_list[iter] = point_clouds_xyzi.points[i].x;
        points_xyzi_list[iter + 1] = point_clouds_xyzi.points[i].y;
        points_xyzi_list[iter + 2] = point_clouds_xyzi.points[i].z;
        points_xyzi_list[iter + 3] = point_clouds_xyzi.points[i].data[3];
    }
    // get semantic result
    std::vector<std::vector<float>> semantic_result = net_->infer(points_xyzi_list, num_points);
    // match the label from label_map and color the point cloud
    for(int i = 0; i < num_points; i++)
    {
        float prob = 0;
        int index = 0;
        for(int k = 0; k < 20; k++)
        {
            if(prob <= semantic_result[i][k])
            {
                prob = semantic_result[i][k];
                index = k;
            }
        }
        labels[i] = net_->getLabel(index);
        labels_prob[i] = prob;
        // get semantic point cloud
        net_->setColorMap(labels[i]);
        point_clouds_rgb.points[i].r = net_->getColorR();
        point_clouds_rgb.points[i].g = net_->getColorG();
        point_clouds_rgb.points[i].b = net_->getColorB();
    }

    return true;
}

bool SumaSLAM::step(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi) {
    if(timestamp_ == 0)
    {
        return initialSystem(point_clouds_xyzi);
    }
    last_frame_ = current_frame_;
    current_frame_ = std::make_shared<Frame>(params_);
    preprocess(point_clouds_xyzi);
    odometry();

    timestamp_++;
    return true;
}

bool SumaSLAM::initialSystem(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi) {
    preprocess(point_clouds_xyzi);

    timestamp_++;
    return true;
}

bool SumaSLAM::odometry() {
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    icp.setInputSource(current_frame_->getPointCloudsPtr());
    icp.setInputTarget(last_frame_->getPointCloudsPtr());

    icp.align(odometry_result_);

    map_->pushBackPose(icp.getFinalTransformation());

    return true;
}

