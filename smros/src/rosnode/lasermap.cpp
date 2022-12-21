//
// Created by tongda on 2022/4/25.
//

#include "SemanticMap.h"

int main(int argc, char** argv)
{
    ros::init (argc, argv, "semantic_map");
    SemanticMap semanticMap;
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_sub = nh.subscribe("surfel", 1000,
                                                   &SemanticMap::callBackPointCloud, &semanticMap);
    ros::Subscriber odometry_sub = nh.subscribe("local_pose", 1000,
                                                   &SemanticMap::callBackPose, &semanticMap);
    ros::Subscriber keyframe_sub = nh.subscribe("keyframe_id", 1000,
                                                   &SemanticMap::callBackLoopClosure, &semanticMap);
    ros::Rate r(10);
    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}