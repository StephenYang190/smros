//
// Created by tongda on 2022/4/24.
//

#include "perception.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl_conversions/pcl_conversions.h>

const float PI = acos(-1);

Perception::Perception()
        : input_point_cloud_(new pcl::PointCloud<pcl::PointXYZI>), img_nh_(nh_) {
    nh_.getParam("work_directory", work_directory_);
    // init rangenet
    std::string model_path;
    if (nh_.getParam("model_path", model_path)) {
        model_path = work_directory_ + model_path;
        net_ = std::make_shared<RangenetAPI>(model_path);
    }
    // get vertex map parameters
    nh_.getParam("width", width_);
    nh_.getParam("height", height_);
    nh_.getParam("fov_down", fov_down_);
    nh_.getParam("fov_up", fov_up_);
    nh_.getParam("max_distance", max_distance_);
    nh_.getParam("min_distance", min_distance_);
    nh_.getParam("line", linenum);
    nh_.getParam("is_remove_vehicle", remove_vehicle_);
    nh_.getParam("is_down_sampling", downsample_);
    fov_ = abs(fov_up_) + abs(fov_down_);
    // init vertex map
    vertex_maps_.resize(width_);
    for (int i = 0; i < width_; i++) {
        vertex_maps_[i].resize(height_);
    }
    // init publisher
    surfel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("surfel", 1000);
    range_image_pub_ = img_nh_.advertise("range_image", 1000);
    // set call back to process point cloud
    point_cloud_sub_ = nh_.subscribe("point_cloud_in", 1000,
                                     &Perception::PointCloudCallback, this);
}

void Perception::PointCloudCallback(const sensor_msgs::PointCloud2 &msg) {
    pcl::fromROSMsg(msg, *input_point_cloud_);
    auto num_points = input_point_cloud_->size();
    // store point cloud in vector type
    std::vector<float> points_xyzi_list(num_points * 4);
    // transform point cloud to vector
    for (int i = 0; i < num_points; i++) {
        int iter = i * 4;
        points_xyzi_list[iter] = input_point_cloud_->points[i].x;
        points_xyzi_list[iter + 1] = input_point_cloud_->points[i].y;
        points_xyzi_list[iter + 2] = input_point_cloud_->points[i].z;
        points_xyzi_list[iter + 3] = input_point_cloud_->points[i].intensity;
    }
    // get semantic result
    std::vector<std::vector<float>> semantic_result =
            net_->infer(points_xyzi_list, num_points);
    std::vector<int> point_labels(num_points);
    for (int i = 0; i < num_points; i++) {
        float prob = 0;
        int label_index = 0;
        for (int k = 0; k < 20; k++) {
            if (prob <= semantic_result[i][k]) {
                prob = semantic_result[i][k];
                label_index = k;
            }
        }
        point_labels[i] = net_->getLabel(label_index);
    }
    // generate surfel
    generateVertexMap(point_labels);
    point2Surfel(semantic_result, msg.header.seq);
    generateNormal();
    publishRangeImage();

    pcl::PointCloud<SemanticSurfel> output_point_cloud;
    output_point_cloud.resize(output_point_number_);
    int i = 0;
    for (int u = 0; u < width_; u++) {
        for (int v = 0; v < height_; v++) {
            // no point fit in this way
            if (vertex_maps_[u][v].index < 0) {
                continue;
            }
            output_point_cloud.points[i] = vertex_maps_[u][v].point;
            i++;
        }
    }
    output_point_cloud.header.frame_id = msg.header.frame_id;
    output_point_cloud.header.seq = msg.header.seq;
    sensor_msgs::PointCloud2 surfel_msg;
    pcl::toROSMsg(output_point_cloud, surfel_msg);

    surfel_pub_.publish(surfel_msg);
}

bool Perception::generateVertexMap(std::vector<int> &point_labels) {
    clearVertexMap();
    int num_points = input_point_cloud_->size();
    std::map<int, int> label_numbers;
    std::map<int, bool> label_valid;
    // compute type number
    for (int i = 0; i < num_points; i++) {
        int label = point_labels[i];
        label_numbers[label]++;
    }
    for (auto iter: label_numbers) {
        label_valid[iter.first] = iter.second > 30;
    }
    // compute the mapping from point to map
    float r_xyz;
    int u, v;
    for (int i = 0; i < num_points; i++) {
        // remove moving object
        if (point_labels[i] < 40 || point_labels[i] > 90)
            continue;
        // skip type with little points
        int label = point_labels[i];
        if (!label_valid[label]) {
            continue;
        }

        if (!computeUVIndex(i, u, v, r_xyz))
            continue;
        // select the nearest point
        if (vertex_maps_[u][v].index != -1 && vertex_maps_[u][v].radius < r_xyz) {
            continue;
        }
        vertex_maps_[u][v].radius = r_xyz;
        vertex_maps_[u][v].index = i;
        vertex_maps_[u][v].point.label = label;
    }

    return true;
}

bool Perception::computeUVIndex(int point_index, int &u, int &v, float &r_xyz) {
    float x, y, z;
    float yaw_angle, pitch_angle;
    x = input_point_cloud_->points[point_index].x;
    y = input_point_cloud_->points[point_index].y;
    z = input_point_cloud_->points[point_index].z;
    // compute the distance to zero point
    r_xyz = sqrt(x * x + y * y + z * z);
    // remove point which is to close
    if (r_xyz < min_distance_ || r_xyz > max_distance_) {
        return false;
    }
    // compute the yaw angle (max:360)
    yaw_angle = atan2(y, x);
    // compute u coordination
    u = (int) (width_ * 0.5 * (yaw_angle / PI + 1));
    // compute the pitch angle (max:360)
    pitch_angle = asin(z / r_xyz) * 180.0 / PI;
    // compute v coordination
    v = (int) (height_ * ((abs(fov_up_) - pitch_angle) / fov_));
    // coordination can not out of range
    u = std::max(std::min(u, width_ - 1), 0);
    v = std::max(std::min(v, height_ - 1), 0);
    return true;
}

void Perception::clearVertexMap() {
    for (int u = 0; u < width_; u++) {
        for (int v = 0; v < height_; v++) {
            vertex_maps_[u][v].index = -1;
        }
    }
}

void Perception::point2Surfel(std::vector<std::vector<float>> &semantic_result,
                              int timestamp) {
    output_point_number_ = 0;
    // construct point
    std::vector<cv::Vec3b> color_mask = net_->getLabels(semantic_result, input_point_cloud_->size());
    for (int u = 0; u < width_; u++) {
        for (int v = 0; v < height_; v++) {
            // no point fit in this site
            if (vertex_maps_[u][v].index < 0) {
                continue;
            }
            int index = vertex_maps_[u][v].index;
            auto &point = vertex_maps_[u][v].point;
            pcl::copyPoint(input_point_cloud_->points[index], point);
            // set label
            point.r = color_mask[index][0];
            point.g = color_mask[index][1];
            point.b = color_mask[index][2];
            point.confidence = initial_confidence_;
            output_point_number_++;
        }
    }

}

void Perception::publishRangeImage() {
    cv::Mat range_image(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int u = 0; u < width_; u++) {
        for (int v = 0; v < height_; v++) {
            if (vertex_maps_[u][v].index > -1) {
                range_image.at<cv::Vec3b>(v, u)[0] = vertex_maps_[u][v].point.b * 255;
                range_image.at<cv::Vec3b>(v, u)[1] = vertex_maps_[u][v].point.g * 255;
                range_image.at<cv::Vec3b>(v, u)[2] = vertex_maps_[u][v].point.r * 255;
            }
        }
    }
    sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", range_image).toImageMsg();
    range_image_pub_.publish(msg);
}

void Perception::generateNormal() {
    for (int u = 0; u < width_; u++) {
        for (int v = 0; v < height_; v++) {
            // no point fit in this way
            if (vertex_maps_[u][v].index < 0) {
                continue;
            }
            // find nearest point
            int u_index = (u + 1) % width_, v_index = (v + 1) % height_;
            if (vertex_maps_[u_index][v].index != -1) {
                continue;
            }
            if (vertex_maps_[u][v_index].index != -1) {
                continue;
            }
            // get the point
            Eigen::Vector3f u1(vertex_maps_[u_index][v].point.x,
                               vertex_maps_[u_index][v].point.y,
                               vertex_maps_[u_index][v].point.z);
            Eigen::Vector3f v1(vertex_maps_[u][v_index].point.x,
                               vertex_maps_[u][v_index].point.y,
                               vertex_maps_[u][v_index].point.z);
            Eigen::Vector3f point_xyz(vertex_maps_[u][v].point.x,
                                      vertex_maps_[u][v].point.y,
                                      vertex_maps_[u][v].point.z);
            // compute normal
            Eigen::Vector3f normal = (u1 - point_xyz).cross(v1 - point_xyz);
            normal.normalize();
            // compute radius
            float molecular = sqrt(2.0) * vertex_maps_[u][v].radius * p_;
            double dis = -1.0 * point_xyz.dot(normal) / vertex_maps_[u][v].radius;
            float denominator = std::min(1.0, std::max(dis, 0.5));
            // set surfel
            vertex_maps_[u][v].point.radius = molecular / denominator;
            vertex_maps_[u][v].point.normal_x = normal[0];
            vertex_maps_[u][v].point.normal_y = normal[1];
            vertex_maps_[u][v].point.normal_z = normal[2];
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pre_process");
    Perception perception;
    ros::spin();
    return 0;
}