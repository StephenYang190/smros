//
// Created by tongda on 2022/4/24.
//

#include "kittireader.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

void read_filelists(const std::string &dir_path,
                    std::vector<std::string> &out_filelsits, std::string type) {
  struct dirent *ptr;
  DIR *dir;
  dir = opendir(dir_path.c_str());
  out_filelsits.clear();
  while ((ptr = readdir(dir)) != NULL) {
    std::string tmp_file = ptr->d_name;
    if (tmp_file[0] == '.')
      continue;
    if (type.size() <= 0) {
      out_filelsits.push_back(ptr->d_name);
    } else {
      if (tmp_file.size() < type.size())
        continue;
      std::string tmp_cut_type =
          tmp_file.substr(tmp_file.size() - type.size(), type.size());
      if (tmp_cut_type == type) {
        out_filelsits.push_back(ptr->d_name);
      }
    }
  }
}

bool computePairNum(std::string pair1, std::string pair2) {
  return pair1 < pair2;
}

void sort_filelists(std::vector<std::string> &filists, std::string type) {
  if (filists.empty())
    return;

  std::sort(filists.begin(), filists.end(), computePairNum);
}

KittiReader::KittiReader() {
  // init publisher
  point_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_in", 1000);
  // get parameters from ros parameter server
  nh_.getParam("work_directory", work_directory_);
  nh_.getParam("pcd_in_path", pcd_in_path_);
  nh_.getParam("sequence", sequence_);
  pcd_in_path_ = work_directory_ + pcd_in_path_ + sequence_ + "/";
  // generate file list
  read_filelists(pcd_in_path_, pcd_lists_, "pcd");
  sort_filelists(pcd_lists_, "pcd");
  // setting traversal index
  current_index_ = 0;
}

bool KittiReader::readPointCloud() {
  if (current_index_ < pcd_lists_.size()) {
    std::string pcd_name = pcd_lists_[current_index_];
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_in_path_ + pcd_name, pointcloud);

    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(pointcloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = "velodyne";
    point_cloud_msg.header.seq = current_index_;
    point_cloud_pub_.publish(point_cloud_msg);

    current_index_++;
    return true;
  }
  return false;
}
void KittiReader::run() {
  while (ros::ok()) {
    ros::Rate r(10);
    if (!readPointCloud()) {
      break;
    }
    r.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "point_reader");
  KittiReader kittireader;
  kittireader.run();
  return 0;
}