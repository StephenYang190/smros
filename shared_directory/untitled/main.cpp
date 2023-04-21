#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigen>

#include "point2plane.h"
#include "nonliner.h"

using pointT = pcl::PointXYZRGB;

int main() {
    std::cout << "Hello, World!" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);

    // read input point
    std::string pcd_path = "/smros_ws/catkin_ws/src/dataset/pcd_dataset/00/000000.pcd";
    pcl::PointCloud<pcl::PointXYZI> input_point;
    pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, input_point);
    // add color to source point
    pcl::PointCloud<pointT> source_point;
    pcl::copyPointCloud(input_point, source_point);
    for (auto &point: source_point.points) {
        point.r = 100;
        point.g = 0;
        point.b = 0;
    }
//    viewer->addPointCloud<pointT>(source_point.makeShared(), "source point");

    // create target pose
//    Eigen::Matrix4f target_pose = Eigen::Matrix4f::Identity();
//
//    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
//    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));
//    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
//
//    Eigen::Matrix3f rotation_matrix(yawAngle * pitchAngle * rollAngle);
//    target_pose.block<3, 3>(0, 0) = rotation_matrix;
//
//    Eigen::Vector3f t(1, 3.5, 0);
//    target_pose.block<3, 1>(0, 3) = t;
//
//    std::cout << "target pose:" << std::endl;
//    std::cout << target_pose << std::endl;

//    // transform point to target
//    pcl::PointCloud<pointT> target_point;
//    pcl::transformPointCloud(source_point, target_point, target_pose);
//    // add color to target point
//    for (auto &point: target_point.points) {
//        point.r = 0;
//        point.g = 100;
//        point.b = 0;
//    }
//    viewer->addPointCloud<pointT>(target_point.makeShared(), "target point");
    // read input point
    std::string pcd_path_1 = "/smros_ws/catkin_ws/src/dataset/pcd_dataset/00/000001.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path_1, input_point);
    // add color to source point
    pcl::PointCloud<pointT> target_point;
    pcl::copyPointCloud(input_point, target_point);
    for (auto &point: target_point.points) {
        point.r = 0;
        point.g = 100;
        point.b = 0;
    }
    viewer->addPointCloud<pointT>(target_point.makeShared(), "target point");
//    std::ofstream fix_out("fix.xyz");
//    for(auto& point : target_point.points)
//    {
//        fix_out << point.x << " " << point.y << " " << point.z << std::endl;
//    }
//    fix_out.close();
//
//    std::ofstream move_out("move.xyz");
//    for(auto& point : source_point.points)
//    {
//        move_out << point.x << " " << point.y << " " << point.z << std::endl;
//    }
//    move_out.close();
    // compute pose
    NonlinearEstimate odometry;
    odometry.SetMaxIterations(4);
    odometry.SetResolution(0.05);
    odometry.SetSourcePointCloud(source_point.makeShared());
    odometry.SetTargetPointCloud(target_point.makeShared());
    pcl::PointCloud<pointT> transform_point;
    viewer->addPointCloud<pointT>(transform_point.makeShared(), "transform point");

    while (!viewer->wasStopped()) {
        if (odometry.IsConverged()) {
            break;
        }
        if (!odometry.Align()) {
            break;
        }
        Eigen::Matrix4f pose = odometry.GetFinalResult();
        // add color to transform point
        pcl::transformPointCloud(source_point, transform_point, pose);
        for (auto &point: transform_point.points) {
            point.r = 0;
            point.g = 0;
            point.b = 100;
        }
        viewer->updatePointCloud(transform_point.makeShared(), "transform point");
        std::cout << pose << std::endl;
        viewer->spinOnce();
    }
    viewer->spin();
//    Eigen::Matrix4f pose = odometry.Align();
//    std::cout << pose << std::endl;
    return 0;
}
