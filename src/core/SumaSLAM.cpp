//
// Created by tongda on 2021/12/16.
//

#include "SumaSLAM.h"

SumaSLAM::SumaSLAM() :
net_(new RangenetAPI())
{

}

bool SumaSLAM::transform(const pcl::PointCloud<pcl::PointXYZI> point_clouds_xyzi,
                         pcl::PointCloud<pcl::PointXYZRGB> &point_clouds_rgb) 
{
    int num_points = point_clouds_xyzi.points.size();
    // store pointcloud in vector type
    std::vector<float> points_xyzi_list(num_points * 4);

    // transform pointcloud to vector and laser scan
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
        int label = net_->getLabel(index);
        labels_prob[i] = prob;
        // get semantic point cloud
        net_->setColorMap(label);
        point_clouds_rgb.points[i].r = net_->getColorR();
        point_clouds_rgb.points[i].g = net_->getColorG();
        point_clouds_rgb.points[i].b = net_->getColorB();
    }
    return false;
}

