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
#include <chrono>
#include "generateObservation.h"

struct State {
    geometry_msgs::Pose robotPose;
    nav_msgs::OccupancyGrid map; //unseen == -1
    nav_msgs::OccupancyGrid houghMask; //value that has been used to weight the graph by walls, used to prevent looping hough walls.
    int mapX,mapY,mapZ;
};
struct adjNode{
    adjNode(){
        N = 0;
        M = 0;
        Q = 0;
        action = -1;
    }
    int nodeNum; //node Number
    bool observationNode;
    State robotState; //state of the system at this node.
    int action;
    std::vector<int> observation;
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
class Graph{
private:
    std::vector<std::vector<int>> adjacencyList;
    std::vector<adjNode> nodeList;
public:
    int numNodes;
    Graph(){
        numNodes = 0;
    }
    int addNode(State nodeState, int action){
        adjNode nodeToAdd;
        nodeToAdd.robotState = nodeState;
        nodeToAdd.nodeNum = numNodes;
        nodeToAdd.action = action;
        nodeToAdd.N = 0;
        nodeToAdd.M = 0;
        nodeToAdd.Q = 0;
        nodeList.push_back(nodeToAdd);
        /*edge edgeToAdd; //NOTES:: Removed v0.01 in favor of vector of vector implementation
        edgeToAdd.node = numNodes-1;
        edgeToAdd.nextNode = nullptr;
        edgeToAdd.prevNode = nullptr;
        listArray.push_back(edgeToAdd); */
        // Start w/ first vector
        std::vector<int> list_into;
        list_into.push_back(numNodes); //Probably will need to fix implementation of node referencing
        adjacencyList.push_back(list_into);
        numNodes++;

        return nodeToAdd.nodeNum;
    }
    adjNode * getNode(int nodeNum){
        for (int i = 0;i<numNodes;i++){
            if (nodeList[i].nodeNum == nodeNum){
                return &nodeList[i];
            }
        }
    }
    void addEdge(int fromNode, int toNode){
        //need to get the current edge in the list.
        /*
        edge head;
        for (int i = 0; i < numNodes; i++){
            int nodeToCheck = listArray[i].node;
            std::cout << nodeToCheck << std::endl;
            //std::cout << listArray[i].node->robotState.robotPose << std::endl;
            if (nodeToCheck == fromNode){
                head = listArray[i];
                break;
            }
        }
        //now have the ll for the from node...
        edge edgeToadd;
        edge currentEdge = head;
        while ( currentEdge.nextNode != NULL){ //iterate to the end of the list.
            currentEdge = *currentEdge.nextNode;
        }

        edgeToadd.prevNode = &currentEdge;
        currentEdge.nextNode = &edgeToadd;
        edgeToadd.node = toNode; //add the edge to the list.
        edgeToadd.nextNode = nullptr;
        listArray.push_back(edgeToadd);
         */
        for (int ii = 0; ii < adjacencyList.size(); ii++){
            //Interate through the vectors
            std::vector<int> tmp_container = adjacencyList[ii];
            if (tmp_container[0] == fromNode){
                adjacencyList[ii].push_back(toNode); //add the connected node to the list
            }
        }
    }

    std::vector<int> getAdjacentNodes(int nodeNumber) {
        std::vector<int> adjacency;
        bool found = false;
        for (int ii = 0; ii < numNodes; ii++) {//Get the head of the  ll for the desired node
            if (adjacencyList[ii][0] == nodeNumber) {
                adjacency = adjacencyList[ii];
                found = true;
                break;
            }
        }
        return adjacency;
    }
};

int actionProgWiden(Graph *tree, int current_node){
    adjNode *currentNode = tree->getNode(current_node);
    int proposedAction = rand() % 4; // generate number between 0 and 3
    //DEBUG::
    proposedAction = 1;
    //Notes: Node we have is going to be an observation Node should have up to four children.
    int newNode;
    std::cout << proposedAction << std::endl; //DEBUG
    std::vector<int> children = tree->getAdjacentNodes(current_node);
    for (int j = 0; j < children.size();j++){
        std::cout<<"Child: " <<children[j]<<std::endl;
    }
    bool new_node = false;
    //std::cout << "Num Kids: " << children.size() << std::endl;
    if ( children.size() == 1){
        //There are no attached nodes to the current node
        std::cout << "No kids" << std::endl;
        //Need to build a new node no matter what
        new_node = true;
        State actionNodeState;
        actionNodeState = currentNode->robotState;
        newNode = tree->addNode(actionNodeState, proposedAction);
        std::cout << "Created Node #: " << newNode <<"\n Connected to : "<< current_node << std::endl;
        tree->addEdge(current_node,newNode);

    } else {
        //Now need to see if the node w/ action already exists
        bool found = false;
        for (int ii = 1; ii < children.size(); ii++){
            //looking through the adjacency vector, the first element is the from node
            adjNode tmp = *(tree->getNode(children[ii]));
            //std::cout << "Tmp Is: " << children[ii] <<"\nTmp Holds: " << tmp.action << std::endl; //DEBUG::
            if (tmp.action == proposedAction){
                newNode = tmp.nodeNum; //Grab the matched node
                found = true;
                tree->getNode(children[ii])->N++; //increment N(ha)
                std::cout << "Found Matching Action Node: " << tmp.nodeNum << std::endl; //DEBUG::
                break;
            }
        }
        if (!found){  //Means there are children but none match the action.
            //Need to build a new node now
            State actionNodeState;
            actionNodeState = currentNode->robotState; //copy node state, not sure if this is needed..... probably isn't
            newNode = tree->addNode(actionNodeState, proposedAction);
            std::cout << "Created Node #: " << newNode <<"\n Connected to : "<< current_node << std::endl;
            tree->addEdge(current_node,newNode);
        }
    }
    //Now increment N(h)
    currentNode->N++;
    return newNode;
}

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
void euler2quat(geometry_msgs::Quaternion *q, EulerAngles RPY){
    // conversion from RPY to quaternions
    // Will put the quaternion into the referenced geometry_msg
    double cy = cos(RPY.yaw * 0.5);
    double sy = sin(RPY.yaw * 0.5);
    double cp = cos(RPY.pitch * 0.5);
    double sp = sin(RPY.pitch * 0.5);
    double cr = cos(RPY.roll * 0.5);
    double sr = sin(RPY.roll * 0.5);

    q->w = cy * cp * cr + sy * sp * sr;
    q->x = cy * cp * sr - sy * sp * cr;
    q->y = sy * cp * sr + cy * sp * cr;
    q->z = sy * cp * cr - cy * sp * sr;
    //Done
}



float forwardSimulate(State *input_state, int action_number){
    //Should return a generated observation, a reward and the next state...

    float x = input_state->robotPose.position.x;
    float y = input_state->robotPose.position.y;

    float action_reward  = -1;
    int mapX = (x - input_state->map.info.origin.position.x) / input_state->map.info.resolution;
    int mapY = (y - input_state->map.info.origin.position.y) / input_state->map.info.resolution;
    //Pack into 2d array for neighbors data
    float temp_map[input_state->map.info.height][input_state->map.info.width];
    int row = 0;
    int col = 0;
    std::cout << "Width:" << input_state->map.info.width << std::endl;
    std::cout << "Height:" << input_state->map.info.height << '\n' << input_state->map.data.size() << std::endl;

    for (int j = 0; j < input_state->map.data.size(); j++){
        temp_map[row][col] = input_state->map.data[j];
        if (col == 100){
            row++;
            col = 0;
        } else {
            col++;
        }
    }

    std::cout << "X Cell: " << mapX << "\nY Cell: "<< mapY <<  std::endl;
    geometry_msgs::Quaternion inputQuat = input_state->robotPose.orientation;
    EulerAngles RPY;
    quat2euler(inputQuat, &RPY);
    geometry_msgs::Pose u_t;
    std::cout << RPY.yaw << std::endl;
    //Now get the action ides
    EulerAngles yaw_change;
    yaw_change.yaw = 0;
    std::cout << yaw_change.roll << std::endl;
    int dist = 0;
    switch (action_number) {
        case 0: dist = 1;
                action_reward = 1;
            break;
        case 1: RPY.yaw = RPY.yaw - M_PI / 2;
                yaw_change.yaw = -1 * M_PI / 2;
            break;
        case 2: RPY.yaw = RPY.yaw + M_PI / 2;
                 yaw_change.yaw = M_PI / 2;
            break;
        case 3: dist = 2;
                action_reward = 0;
            break;
        case -1: std::cout << "Failed to get correct node!" << std::endl;
        default: std::cout << "Never should be here. Code 2" << std::endl;
    }
    if (RPY.yaw > 2 * M_PI ){ //bound fixup
        RPY.yaw -= 2 * M_PI;
    } else if (RPY.yaw < 0){
        RPY.yaw += 2 * M_PI;
    }
    std::cout << RPY.yaw << std::endl;
    //Now the robot it pointed in the correct direction, but need to move in pointed direction.
    geometry_msgs::Quaternion q_u;
    euler2quat(&q_u, yaw_change );
    u_t.position.x = std::cos(RPY.yaw)*dist;
    u_t.position.y = std::sin(RPY.yaw)*dist; //change in pose stored
    u_t.position.z = 0;
    u_t.orientation = q_u;
    //std::cout << u_t.orientation.w <<" : " << u_t.orientation.z << std::endl;

}

float simulate(State stateIn, Graph tree , int d, int current_node) {
    //returns the total reward for the specified level of recursion
    //Normal Vars:
    double k_0 = 6.0;
    double alpha_0 = 1.0/5.0;

    if (d == 0){
        return 0;
    }
    int action_node_number = actionProgWiden(&tree,current_node);
    std::vector<int> observationChildren = tree.getAdjacentNodes(action_node_number);
    adjNode * actionNode = tree.getNode(action_node_number);
    int N_ha = actionNode->N;
    int action_value = actionNode->action;
    if (observationChildren.size() <= k_0*pow(N_ha,alpha_0)){ //means we should make a new node...probably
        float q = forwardSimulate(&(actionNode->robotState), action_value);

    }

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
    //std::default_random_engine gene (453215);

    int n = 100; //Number of tree particles to generate.
    Graph tree;
    tree.addNode(initialstate,-1);
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
    std::default_random_engine gene (1908);
    std::srand(std::time(NULL));
    std::cout << "test" << std::endl;
    ros::init(argc, argv, "mc_plan");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("mc_plan", plan);

    //ros::spin();
    //DEBUG BELOW::::
    //Ideal: Call planner below first. May need to init fake stuff.
    State initialState;
    geometry_msgs::Pose initPose;
    initPose.position.z = 0;
    initPose.position.x = 10;
    initPose.position.y = 10;
    std::cout << initPose.orientation << std::endl;
    initialState.robotPose = initPose;
    //float vall = simulate(stateIn, tree, explorationDepth,currentNode);
    Graph testTree;
    testTree.addNode(initialState, -1);
    std::cout << testTree.getNode(0)->robotState.robotPose << std::endl;
    initialState.robotPose.position.x = -10;
    initialState.robotPose.position.y = -10;

    testTree.addNode(initialState, -1);
    std::cout << testTree.getNode(1)->robotState.robotPose << std::endl;
    testTree.addEdge(0,1);


    testTree.addNode(initialState, -1);
    testTree.addEdge(0,0);
    testTree.addEdge(1,2);


    int newNode = actionProgWiden(&testTree,2);
    newNode = actionProgWiden(&testTree,2);

    std::cout << "Created Node: " << newNode <<" \n With Action: "<< testTree.getNode(newNode)->action << std::endl;
    std::cout << testTree.getNode(2)->N << std::endl;

    //Now Working on Simulate::
    initPose.position.z = 0;
    initPose.position.x = 10;
    initPose.position.y = 10;
    initialState.robotPose = initPose;
    Graph simulateTree;
    simulateTree.addNode(initialState,-1);
/*    geometry_msgs::Quaternion testQuat;
    testQuat.w = 1;
    testQuat.x = 0.;
    testQuat.y = 0;
    testQuat.z = 0;*/

    //EulerAngles result;
    //quat2euler(testQuat, &result);
    //std::cout << "angles: " << result.roll << '\n' << result.pitch << '\n' << result.yaw << std::endl;

    //float Q = simulate(initialState,simulateTree,10,0);

    //DEBUG:::: Building stuff for testing
    nav_msgs::OccupancyGrid testing_map;
    testing_map.info.resolution = 0.5;
    testing_map.info.height = 100;
    testing_map.info.width = 100;

    std::cout << testing_map.info.width << std::endl;
    std::vector<int8_t> testMap;
    std::vector<int> obsMap;
    for (int j = 0; j < 10000; j++){
        testMap.push_back(40);
        obsMap.push_back(j / 100);
    }
    //std::cout << "check: " << (int) testMap[0] << std::endl;
    //testing_map.d
    testing_map.data = testMap;
    State initial_state;
    initial_state.map = testing_map;
    initial_state.robotPose = initPose;
    initial_state.robotPose.orientation.w = 1;

    forwardSimulate(&initial_state,1);
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<int> testObs = generate_observation(obsMap);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    int sum = 0;
    for (int j = 0; j < testObs.size(); j++){
        sum += testObs[j];
    }
    std::cout << sum<< std::endl;
    return 0;
}


