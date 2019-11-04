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
#include <random>
#include <iostream>
#include <vector>

struct State {
    geometry_msgs::Pose robotPose;
    nav_msgs::OccupancyGrid map; //unseen == -1
    nav_msgs::OccupancyGrid houghMask; //value that has been used to weight the graph by walls, used to prevent looping hough walls.
    int mapX,mapY,mapZ;
};
struct adjNode{
    int nodeNum; //node Number
    State robotState; //state of the system at this node.
    int action;
};
struct edge{
    adjNode* node;
    edge* prevNode;
    edge* nextNode;
};
class Graph{
private:
    std::vector<adjNode> nodeList;
    std::vector<edge> listArray;
    int numNodes;
public:
    Graph(){
        numNodes = 0;
    }
    void addNode(State nodeState){
        adjNode nodeToAdd;
        nodeToAdd.robotState = nodeState;
        nodeToAdd.nodeNum = numNodes;
        nodeToAdd.action = -1;
        numNodes++;
        nodeList.push_back(nodeToAdd);
        edge edgeToAdd;
        edgeToAdd.node = &nodeToAdd;
        edgeToAdd.nextNode = nullptr;
        edgeToAdd.prevNode = nullptr;
        listArray.push_back(edgeToAdd);
    }
    void addEdge(int fromNode, adjNode *toNode){
        //need to get the current edge in the list.
        edge head;
        for (int i = 0; i < numNodes; i++){
            int nodeToCheck = listArray[i].node->nodeNum;
            if (nodeToCheck == fromNode){
                head = listArray[i];
                break;
            }
        }
        //now have the ll for the from node...
        edge edgeToadd;
        edge currentEdge = head;
        while ( currentEdge.nextNode != nullptr){ //iterate to the end of the list.
            currentEdge = *currentEdge.nextNode;
        }
        edgeToadd.prevNode = &currentEdge;
        currentEdge.nextNode = &edgeToadd;
        edgeToadd.node = toNode; //add the edge to the list.
    }
    adjNode getNode(int nodeNum){
        for (int i = 0;i<numNodes;i++){
            if (nodeList[i].nodeNum == nodeNum){
                return nodeList[i];
            }
        }
    }


};
geometry_msgs::Pose actionProgWiden(Graph tree, int current_node){
    adjNode currentNode = tree.getNode(current_node);
    int proposedAction = rand() % 4; // generate number between 0 and 3
    //Notes: Node we have is going to be an observation Node should have up to four childen.

}

float simulate(State stateIn, Graph tree , int d, int current_node) {
    //returns the total reward for the specified level of recursion
    if (d == 0){
        return 0;
    }
    geometry_msgs::Pose action = actionProgWiden(tree,current_node);

}
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
    int gridZ = roundf(currentCellZ);

    State initialstate;
    initialstate.map = gridIn;
    initialstate.mapX = gridX;
    initialstate.mapY = gridY;
    initialstate.mapZ = gridZ;
    initialstate.robotPose = poseIn;

    //Plan Procedure
    std::default_random_engine gene (453214);

    int n = 100; //Number of tree particles to generate.
    Graph tree;
    tree.addNode(initialstate);
    for (int j = 1; j < n; j++){


    }
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


