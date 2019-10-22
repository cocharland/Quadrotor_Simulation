//
// Created by cody on 10/22/19.
//
#include "ros/ros.h"
#include "quadrotor_sim/mc_plan.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <math.h>



bool plan(quadrotor_sim::mc_plan::Request  &req,
         quadrotor_sim::mc_plan::Response &res)
{
    nav_msgs::OccupancyGrid gridIn = req.ProjectedGrid;
    geometry_msgs::Pose poseIn = req.currentPose;

    float resolution_map = gridIn.info.resolution;
    geometry_msgs::Pose mapOrigin = gridIn.info.origin;

    //need to calculate current cell:
    float currentCellX = (poseIn.position.x - mapOrigin.position.x)/resolution_map;
    float currentCellY = (poseIn.position.y - mapOrigin.position.y)/resolution_map;
    float currentCellZ = (poseIn.position.z - mapOrigin.position.z)/resolution_map;

    int gridX = roundf(currentCellX);
    int gridY = roundf(currentCellY);


    //check the neighbors for occupancy

    for (int i = 0; i < 4; i++){
        for(int j = 0; j<4; j++){
            ROS_INFO("Cell: [%i]",gridX);
        }
    }


    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mc_plan");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("mc_plan", plan);

    ros::spin();



    return 0;
}


