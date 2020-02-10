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
#include <tf2_ros/transform_listener.h>

geometry_msgs::Pose currentPose;
geometry_msgs::Pose currentCommandedPose;
bool inService = false;
ros::NodeHandle * nPtr;
ros::Publisher * nPubPtr;

struct EulerAngles{
    double roll, pitch, yaw;
    EulerAngles(){ //construct to 0
        roll = 0;
        pitch = 0;
        yaw = 0;
    }
};

void quat2euler(geometry_msgs::Quaternion q, EulerAngles * RPY){
    double sinr_cosp = 2*(q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    RPY->roll = std::atan2(sinr_cosp,cosr_cosp);

    //pitch
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if ( std::abs(sinp) >= 1){
        RPY->pitch = std::copysign(M_PI / 2, sinp);
    } else {
        RPY->pitch = std::asin(sinp);
    }
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    RPY->yaw = std::atan2(siny_cosp,cosy_cosp);
}

void storePose(const nav_msgs::Odometry::ConstPtr&  msg){
    geometry_msgs::Pose inbound = msg->pose.pose;
    currentPose = inbound;
    tf2_ros::Buffer tfBuff;
    tf2_ros::TransformListener tfListen(tfBuff);
    geometry_msgs::TransformStamped tfStamped;
    while(1) {

        try {
            tfStamped = tfBuff.lookupTransform("base_link", "world", ros::Time(0));
            break;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            continue;
        }
    }
    geometry_msgs::Twist message_out;
    float currentZ = inbound.position.z;
    float currentY = inbound.position.y;
    float currentX = inbound.position.x;
    EulerAngles RPY;
    quat2euler(inbound.orientation,&RPY);
    EulerAngles comRPY;
    quat2euler(currentCommandedPose.orientation,&comRPY);
    message_out.linear.z = 0.1*(currentCommandedPose.position.z-currentZ);
    message_out.linear.x = 0.1*(currentCommandedPose.position.x-currentX);
    message_out.linear.y = 0.1*(currentCommandedPose.position.y-currentY);
    message_out.angular.z = 0.1*atan2(tfStamped.transform.translation.y,tfStamped.transform.translation.x);

    (*nPubPtr).publish(message_out);


}

bool add(quadrotor_sim::move_quad::Request  &req,
         quadrotor_sim::move_quad::Response &res)
{
    inService = true;
    res.success = true;
    std::string poseString;
    geometry_msgs::Pose mapPose = req.commandedPose;
    //now need to transform into local base frame


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
    ROS_INFO("Ready to move");

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
