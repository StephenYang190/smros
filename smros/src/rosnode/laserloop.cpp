//
// Created by tongda on 2022/4/26.
//
#include "LoopClosureDetection.h"

int main(int argc, char** argv)
{
    ros::init (argc, argv, "loopclousuredetection");
    LoopClosureDetection loopClosureDetection;
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_sub = nh.subscribe("surfel", 1000,
                                                   &LoopClosureDetection::callBackPointCloud, &loopClosureDetection);
    ros::Subscriber odometry_sub = nh.subscribe("local_pose", 1000,
                                                &LoopClosureDetection::callBackOdometry, &loopClosureDetection);
    ros::Rate r(10);
    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
