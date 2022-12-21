//
// Created by tongda on 2022/4/24.
//

#include "Preprocess.h"

Preprocess::Preprocess() :
    point_cloud_(new pcl::PointCloud<pcl::PointXYZI>)
{
    // init rangenet
    std::string model_path;
    if(nh_.getParam("model_path", model_path))
    {
        net_ = std::make_shared<RangenetAPI>(model_path);
    }
    // init vertex map
    current_frame_ = std::make_shared<VertexMap>(0);
    // init publisher
    surfel_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("surfel", 1000);
    // get parameters from ros parameter server
    nh_.getParam("is_remove_vehicle", remove_vehicle_);
    nh_.getParam("is_down_sampling", downsample_);

}

void Preprocess::callBack(const sensor_msgs::PointCloud2 &msg) {
    pcl::fromROSMsg(msg, *point_cloud_);
    int num_points = point_cloud_->size();
    // store point cloud in vector type
    std::vector<float> points_xyzi_list(num_points * 4);
    // transform point cloud to vector
    for(int i = 0; i < num_points; i++)
    {
        int iter = i * 4;
        points_xyzi_list[iter] = point_cloud_->points[i].x;
        points_xyzi_list[iter + 1] = point_cloud_->points[i].y;
        points_xyzi_list[iter + 2] = point_cloud_->points[i].z;
        points_xyzi_list[iter + 3] = point_cloud_->points[i].data[3];
    }
    // get semantic result
    std::vector<std::vector<float>> semantic_result = net_->infer(points_xyzi_list, num_points);
    // set point clouds
    current_frame_->setPointCloud(point_cloud_);
    // get point clouds in surfel base
    pcl::PointCloud<Surfel>::Ptr point_clouds_surfel = current_frame_->getPointCloudsPtr();
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
        int label = net_->getLabel(index);
        // set label
        net_->setColorMap(label);
        point_clouds_surfel->points[i].point_type = label;
        point_clouds_surfel->points[i].r = net_->getColorR() / 256.0f;
        point_clouds_surfel->points[i].g = net_->getColorG() / 256.0f;
        point_clouds_surfel->points[i].b = net_->getColorB() / 256.0f;
    }
    // generate surfel
    if(remove_vehicle_)
    {
        current_frame_->removeVehiclePoint();
    }
    current_frame_->points2Surfel(1, downsample_);

    sensor_msgs::PointCloud2 surfel_msg;
    pcl::toROSMsg(*point_clouds_surfel, surfel_msg);
    surfel_msg.header.frame_id = "velodyne";
    surfel_msg.header.seq = msg.header.seq;

    surfel_pub_.publish(surfel_msg);
}
