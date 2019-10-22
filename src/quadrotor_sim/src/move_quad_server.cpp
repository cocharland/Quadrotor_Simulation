#include "ros/ros.h"
#include "quadrotor_sim/move_quad.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "hector_uav_msgs/EnableMotors.h"

geometry_msgs::Pose currentPose;
geometry_msgs::Pose currentCommandedPose;
bool inService = false;
ros::NodeHandle * nPtr;
ros::Publisher * nPubPtr;

void storePose(const nav_msgs::Odometry::ConstPtr&  msg){
    geometry_msgs::Pose inbound = msg->pose.pose;
    currentPose = inbound;

    geometry_msgs::Twist message_out;
    float currentZ = inbound.position.z;
    float currentY = inbound.position.y;
    float currentX = inbound.position.x;

    message_out.linear.z = 0.1*(currentCommandedPose.position.z-currentZ);
    message_out.linear.x = 0.1*(currentCommandedPose.position.x-currentX);
    message_out.linear.y = 0.1*(currentCommandedPose.position.y-currentY);

    (*nPubPtr).publish(message_out);


}

bool add(quadrotor_sim::move_quad::Request  &req,
         quadrotor_sim::move_quad::Response &res)
{
    inService = true;
    res.success = true;
    std::string poseString;
    currentCommandedPose = req.commandedPose;
    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_quad_server");
    ros::NodeHandle n;
    nPtr = &n;
    geometry_msgs::Pose tmp;
    tmp.position.z = 1;
    tmp.position.y = 0;
    tmp.position.x = 0;
    currentCommandedPose = tmp;
    ros::ServiceServer service = n.advertiseService("move_quad", add);
    ROS_INFO("Ready to move this bitch!");

    ros::Subscriber poseSub = n.subscribe("/ground_truth/state",1,storePose);
    ros::Publisher twistPub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    nPubPtr = &twistPub;
    ros::ServiceClient enable_motor_client = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    hector_uav_msgs::EnableMotors enableSrv;
    enableSrv.request.enable = true;
    enable_motor_client.call(enableSrv);



    ros::spin();

    return 0;
}
