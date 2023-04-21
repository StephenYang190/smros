//
// Created by tongda on 2022/4/24.
//

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "RangenetAPI.hpp"
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>

const float PI = acos(-1);

int main(int argc, char **argv) {
    std::cout << "Hello, World!" << std::endl;
    // init rangenet
    std::string model_path = "/smros_ws/catkin_ws/src/model/darknet53/";
    std::shared_ptr<RangenetAPI> net_ = std::make_shared<RangenetAPI>(model_path);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);

    // read input point from pcd
    std::string pcd_path = "/smros_ws/catkin_ws/src/dataset/pcd_dataset/00/000000.pcd";
    pcl::PointCloud<pcl::PointXYZI> input_point;
    pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, input_point);
    std::cout << "point number in pcd: " << input_point.points.size() << std::endl;
    // store point cloud in vector type
    int input_numbers = input_point.size();
    std::vector<float> points_from_pcd(input_numbers * 4);
    for (int i = 0; i < input_numbers; i++) {
        int iter = i * 4;
        points_from_pcd[iter] = input_point.points[i].x;
        points_from_pcd[iter + 1] = input_point.points[i].y;
        points_from_pcd[iter + 2] = input_point.points[i].z;
        points_from_pcd[iter + 3] = input_point.points[i].intensity;
    }

//    std::ifstream in("/smros_ws/point0.txt");
//    int num_points;
//    in >> num_points;
//    std::cout << "point number in txt: " << num_points / 4 << std::endl;
//    std::vector<float> points_xyzi_list(num_points);
//    for (int i = 0; i < num_points; i++) {
//        float value;
//        in >> value;
//        points_xyzi_list[i] = value;
//    }
//    // match
//    pcl::KdTreeFLANN<pcl::PointXYZI> kdTreeFlann;
//    kdTreeFlann.setInputCloud(input_point.makeShared());
//    std::map<int, int> match_result;
//    std::vector<float> distance;
//    std::vector<float> x_distance, y_distance, z_distance, inte_distance;
//    int K = 1;
//    std::vector<int> pointIdxKNNSearch(K);
//    std::vector<float> pointKNNSquaredDistance(K);
//    for (int i = 0; i < num_points / 4; i++) {
//        pcl::PointXYZI point;
//        int iter = i * 4;
//        point.x = points_xyzi_list[iter];
//        point.y = points_xyzi_list[iter + 1];
//        point.z = points_xyzi_list[iter + 2];
//        point.data[3] = points_xyzi_list[iter + 3];
//        if (kdTreeFlann.nearestKSearch(point, K, pointIdxKNNSearch,
//                                       pointKNNSquaredDistance) > 0) {
//            match_result[i] = pointIdxKNNSearch[0];
//            auto in_point = input_point.points[pointIdxKNNSearch[0]];
//            double x_dis = point.x - in_point.x;
//            double y_dis = point.y - in_point.y;
//            double z_dis = point.z - in_point.z;
//            double inten_dis = point.data[3] - in_point.data[3];
//            x_distance.push_back(x_dis);
//            y_distance.push_back(y_dis);
//            z_distance.push_back(z_dis);
//            inte_distance.push_back(inten_dis);
//            distance.push_back(pointKNNSquaredDistance[0]);
//        }
//    }
//    std::ofstream out("/smros_ws/shared_directory/pair.csv");
//    int i = 0;
//    for (auto iter: match_result) {
//        out << iter.first << "," << iter.second << "," << distance[i] << ",";
//        out << x_distance[i] << "," << y_distance[i] << "," << z_distance[i] << ",";
//        out << inte_distance[i] << std::endl;
//        i++;
//    }
//    out.close();
//    // transform point cloud to vector
//    int i = 0;
//    for (int v = 0; v < vertexMap.height_; v++) {
//        for (int u = 0; u < vertexMap.width_; u++) {
//
//            if (vertexMap.vertex_maps_[u][v].index != -1) {
//                auto point = vertexMap.input_point->points[vertexMap.vertex_maps_[u][v].index];
//                int iter = i * 4;
//                points_xyzi_list[iter] = point.x;
//                points_xyzi_list[iter + 1] = point.y;
//                points_xyzi_list[iter + 2] = point.z;
//                points_xyzi_list[iter + 3] = point.data[3];
//                i++;
//            }
//        }
//    }
//    for (int i = 0; i < num_points; i++) {
//        int iter = i * 4;
//        points_xyzi_list[iter] = input_point.points[i].x;
//        points_xyzi_list[iter + 1] = input_point.points[i].y;
//        points_xyzi_list[iter + 2] = input_point.points[i].z;
//        points_xyzi_list[iter + 3] = input_point.points[i].data[3];
//    }
    // get semantic result
    std::vector<std::vector<float>> semantic_result =
            net_->infer(points_from_pcd, input_numbers);
    std::vector<cv::Vec3b> color_mask = net_->getLabels(semantic_result, input_numbers);

    pcl::PointCloud<pcl::PointXYZRGB> out_pointcloud;
    // test rangenet
    out_pointcloud.resize(input_numbers);
    for (int i = 0; i < input_numbers; i++) {
        int iter = i * 4;
        out_pointcloud.points[i].x = points_from_pcd[iter];
        out_pointcloud.points[i].y = points_from_pcd[iter + 1];
        out_pointcloud.points[i].z = points_from_pcd[iter + 2];
        out_pointcloud.points[i].r = color_mask[i][0];
        out_pointcloud.points[i].g = color_mask[i][1];
        out_pointcloud.points[i].b = color_mask[i][2];
    }
    viewer->addPointCloud<pcl::PointXYZRGB>(out_pointcloud.makeShared(), "out point");

    viewer->spin();
    return 0;
}