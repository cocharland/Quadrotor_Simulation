#include "ros/ros.h"
#include "quadrotor_sim/move_quad.h"
#include <cstdlib>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "hector_uav_msgs/EnableMotors.h"
#include "quadrotor_sim/mc_plan.h"


ros::ServiceClient * PlanClientPtr;
geometry_msgs::Pose currentPose;

void storePose(const nav_msgs::Odometry::ConstPtr&  msg) {
    geometry_msgs::Pose inbound = msg->pose.pose;
    currentPose = inbound;
}

void get_map(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    nav_msgs::OccupancyGrid gridIn = *msg;
    quadrotor_sim::mc_plan planSrv;
    planSrv.request.ProjectedGrid = gridIn;
    planSrv.request.currentPose = currentPose;
    (*PlanClientPtr).call(planSrv);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_quad_test_client");


    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<quadrotor_sim::move_quad>("move_quad");
    quadrotor_sim::move_quad srv;
    ros::Rate loop_rate(50); //set loop frequency

    //Need to enable motors here
    ros::ServiceClient enable_motor_client = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    hector_uav_msgs::EnableMotors enableSrv;
    enableSrv.request.enable = true;
    enable_motor_client.call(enableSrv);
    //also need to subscribe to :/ground_truth/state....done server side

    ros::ServiceClient plan_client = n.serviceClient<quadrotor_sim::mc_plan>("mc_plan");

    PlanClientPtr = &plan_client;

    ros::Subscriber poseSub = n.subscribe("/ground_truth/state",1,storePose);

    ros::Subscriber mapSub = n.subscribe("/projected_map",1,get_map);


    geometry_msgs::Pose poseToSend;
    poseToSend.position.x = -2;
    poseToSend.position.z = 1;


    srv.request.commandedPose = poseToSend;

    if (client.call(srv))
    {
        ROS_INFO("Moving Quad");
    }
    else
    {
        ROS_ERROR("Failed to call service move_quad");
        return 1;
    }
    ros::spin();
    return 0;
}

