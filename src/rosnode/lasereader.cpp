//
// Created by tongda on 2022/4/24.
//
#include "KittiReader.h"
int main(int argc, char** argv)
{
    ros::init (argc, argv, "point_reader");
    KittiReader kittireader;
    ros::Rate r(10);
    while (ros::ok()){
        kittireader.readPointCloud();
        r.sleep();
    }
    return 0;
}