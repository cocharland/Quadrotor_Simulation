//
// Created by cody on 2/23/20.
//

#include <vector>
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"

#ifndef QUADROTOR_SIM_STRUCTURES_H
#define QUADROTOR_SIM_STRUCTURES_H


struct State {
    geometry_msgs::Pose robotPose;
    nav_msgs::OccupancyGrid map; //unseen == -1; 0-100 belief map of the robot....
    nav_msgs::OccupancyGrid houghMask; //value that has been used to weight the graph by walls, used to prevent looping hough walls.
    int mapX,mapY,mapZ;
};


struct adjNode{
    adjNode(){
        N = 0;
        M = 0;
        Q = 0;
        action = -1;
        color = 0;
        d = -1;
        pi = -1;
    }
    int nodeNum; //node Number
    int d, pi;
    bool observationNode;
    uint8_t color; //0 = white; 1 = grey; 2 = black
    State robotState; //state of the system at this node.
    int action;
    std::vector<int8_t> observation;
    int N,M;
    double Q;

};
struct edge{
    int node;
    edge* prevNode;
    edge* nextNode;
};
struct EulerAngles{
    double roll, pitch, yaw;
    EulerAngles(){ //construct to 0
        roll = 0;
        pitch = 0;
        yaw = 0;
    }
};

#endif //QUADROTOR_SIM_STRUCTURES_H
