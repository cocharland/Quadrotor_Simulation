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
#include <queue>
#include <fstream>
#include "generateObservation.h"
#include "Graph.h"
#include "Structures.h"

bool DEBUG = false;


int actionProgWiden(Graph *tree, int current_node){
    adjNode *currentNode = tree->getNode(current_node);
    int proposedAction = rand() % 4; // generate number between 0 and 3
    //DEBUG::
    proposedAction = 1;
    //Notes: Node we have is going to be an observation Node should have up to four children.
    int newNode;
    //std::cout << proposedAction << std::endl; //DEBUG
    std::vector<int> children = tree->getAdjacentNodes(current_node);
    for (int j = 0; j < children.size();j++){
        std::cout<< "Child: " <<children[j]<<std::endl;
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

void particleFilter(State * robot_state, std::vector<int8_t> observation){
    //Takes a robot state and an observation and pushes updates into the map beliefs

    //First step collate the map into a proper format for neigbor definitions
    int map_limit_x = robot_state->map.info.width;
    int map_limit_y = robot_state->map.info.height;
    std::vector<int8_t> beliefMap = robot_state->map.data;
    int temp_map[map_limit_y][map_limit_x];
    int collated_observation[map_limit_y][map_limit_x];
    int row = 0;
    int col = 0;
    for (int j = 0; j < beliefMap.size(); j++){
        temp_map[row][col] = (int) beliefMap.at(j);
        collated_observation[row][col] = observation.at(j);
        if (col == map_limit_x-1){
            row++;
            col = 0;
        } else {
            col++;
        }
    }
    //TODO::
    // -- Need to updates the temp odd updates to actually use the bayesian update steps
    int index = 0;
    int updateValue = 20;
    int default_value = 40;
    for(int j = 0; j < map_limit_x; j++){
        for(int k = 0; k < map_limit_y; k++){
            int tmp = 0;
            switch (collated_observation[k][j]){ // if the observation includes the cell
                case 0: //We've observed the cell to be open
                    if(temp_map[k][j] == -1){//first observation
                        temp_map[k][j] = default_value;
                    }
                    tmp = temp_map[k][j]-updateValue;
                    if (tmp > 0 ){
                        robot_state->map.data.at(index)=(int8_t)tmp;
                        temp_map[k][j]-=updateValue;
                    } else {
                        robot_state->map.data.at(index)=0;
                    }
                    break;
                case 1: //Observed closed cell
                    if(temp_map[k][j] == -1){//first observation
                        temp_map[k][j] = default_value;
                    }
                    tmp = temp_map[k][j]+updateValue;
                    if (tmp < 100 ){
                        robot_state->map.data.at(index)= (int8_t)tmp;
                        temp_map[k][j] += updateValue;
                    } else {
                        robot_state->map.data.at(index)=100;
                    }
                    break;
                default:
                    break;
            }
            index++;
        }
    }
    std::ofstream testOut;
    testOut.open("TestUpdate.txt");
    for (int row = 0; row < map_limit_y;row++){
        for (int col = 0; col < map_limit_x;col++){
            testOut << std::to_string(temp_map[row][col]);
            if (col < map_limit_x-1){
                testOut << ',';
            }
            //std::cout << std::to_string(resulting_obs[row][col]) << std::endl;
        }
        testOut << '\n';
    }
    testOut.close();
}

float forwardSimulate(State *input_state, int action_number, std::vector<int8_t>  &obsContainer){
    //Should return a generated observation, a reward and the next state...

    float x = input_state->robotPose.position.x;
    float y = input_state->robotPose.position.y;

    float action_reward  = -1;
    int mapX = (x - input_state->map.info.origin.position.x) / input_state->map.info.resolution;
    int mapY = (y - input_state->map.info.origin.position.y) / input_state->map.info.resolution;
    input_state->mapX = mapX;
    input_state->mapY = mapY;

    //std::cout << "X Cell: " << mapX << "\nY Cell: "<< mapY <<  std::endl;
    geometry_msgs::Quaternion inputQuat = input_state->robotPose.orientation;
    EulerAngles RPY;
    quat2euler(inputQuat, &RPY);
    geometry_msgs::Pose u_t;
    //std::cout << RPY.yaw << std::endl;
    //Now get the action ides
    EulerAngles yaw_change;
    yaw_change.yaw = 0;
    //std::cout << yaw_change.roll << std::endl;
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
                break;
        default: std::cout << "Never should be here. Code: 2" << std::endl;
    }
    if (RPY.yaw > 2 * M_PI ){ //bound fixup
        RPY.yaw -= 2 * M_PI;
    } else if (RPY.yaw < 0){
        RPY.yaw += 2 * M_PI;
    }
    //Now the robot is pointed in the correct direction, but need to move in pointed direction.
    geometry_msgs::Quaternion q_u;
    euler2quat(&q_u, yaw_change );
    u_t.position.x = std::cos(RPY.yaw)*dist;
    u_t.position.y = std::sin(RPY.yaw)*dist; //change in pose stored
    u_t.position.z = 0;
    u_t.orientation = q_u;
    // Update map positions to reflect the taken action....
    mapX = (x - input_state->map.info.origin.position.x + u_t.position.x) / input_state->map.info.resolution;
    mapY = (y - input_state->map.info.origin.position.y + u_t.position.y) / input_state->map.info.resolution;
    input_state->mapX = mapX;
    input_state->mapY = mapY;
    //std::cout << u_t.orientation.w <<" : " << u_t.orientation.z << std::endl;
    //Build an observation:
    //generate a ray result over the observation. -1 means no info gained, 0 is seen empty, 1 is seen filled
    std::vector<int8_t> ray_result = rayCast(input_state->map.data,RPY.yaw,mapX,mapY,input_state->map.info.width,input_state->map.info.height,input_state->map.info.resolution);
    obsContainer = ray_result;

    // Need to change the probabilities over the map here:



    //Now: reward mapping
    //rayCast(std::vector<int8_t> beliefMap, float theta, int mapX, int mapY, int map_limit_x, int map_limit_y,double map_resolution)
    int observation_reward = 0;
     for (int j = 0; j < ray_result.size(); j++){
        if (ray_result.at(j) != -1){
            //now we are in the seen area
            //std::cout << std::to_string(input_state->map.data.at(j)) << std::endl;
            if(input_state->map.data.at(j) == -1){
                observation_reward++;
            }
        }
    }
    particleFilter(input_state, ray_result);
    //impact reward
    int impact_reward = 0;
    if (input_state->map.data.at(mapY*input_state->map.info.width+mapX) > 30){
        impact_reward = -100;
    }

    //make sure the states are all rolled up proper

    //Fixup the new x,y positions
    input_state->robotPose.position.x = input_state->map.info.resolution*mapX+input_state->map.info.origin.position.x;
    input_state->robotPose.position.y = input_state->map.info.resolution*mapY+input_state->map.info.origin.position.y;

    euler2quat(&q_u,RPY);
    input_state->robotPose.orientation = q_u;

    return impact_reward+observation_reward+action_reward;

    //Need to map from the vector to a point in the belief map *after* the motion


}

float simulate(State stateIn, Graph *tree , int d, int current_node) {
    //returns the total reward for the specified level of recursion
    //Normal Vars:
    double k_0 = 6.0;
    double alpha_0 = 1.0/5.0;

    if (d == 0){
        return 0;
    }
    int action_node_number = actionProgWiden(tree, current_node);
    std::vector<int> observationChildren = tree->getAdjacentNodes(action_node_number);
    adjNode * actionNode = tree->getNode(action_node_number);
    int N_ha = actionNode->N;
    int action_value = actionNode->action;
    std::vector<int8_t> ray_result;
    if (observationChildren.size() <= k_0*pow(N_ha,alpha_0)){ //means we should make a new node...probably
        float q = forwardSimulate(&(actionNode->robotState), action_value, ray_result);

    }

}
bool plan(quadrotor_sim::mc_plan::Request  &req,
         quadrotor_sim::mc_plan::Response &res)
{
    //Request contains occ grid, and geometry msg current pose
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
    int rootNode = 0;
    for (int j = 1; j < n; j++){
        simulate(initialstate, &tree, 10, rootNode); //buildup the tree

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
    testTree.addEdge(0,2);
    testTree.addEdge(1,2);
    testTree.addNode(initialState,1);
    testTree.addEdge(0,3);
    testTree.addEdge(1,5);
    testTree.addEdge(5,6);

    std::vector<int> adjacency_list = testTree.getAdjacentNodes(0);

    for (int j =0; j < adjacency_list.size();j++){
        std::cout << adjacency_list.at(j) << std::endl;
    }
    testTree.removeEdge(0,1);
    adjacency_list = testTree.getAdjacentNodes(0);
    std::cout << "------" << std::endl;
    for (int j = 0; j < adjacency_list.size(); j++){
        std::cout << adjacency_list.at(j) << std::endl;
    }
    testTree.prune_tree(0);

    //float res = forwardSimulate(initialState,0);
    //forwardSimulate(&initialState,0);
    //return 0;

    int newNode = actionProgWiden(&testTree,2);
    newNode = actionProgWiden(&testTree,2);

    std::cout << "Created Node: " << newNode <<" \n With Action: "<< testTree.getNode(newNode)->action << std::endl;
    std::cout << testTree.getNode(2)->N << std::endl;

    //Now Working on Simulate::
    initPose.position.z = 0;
    initPose.position.x = 10;
    initPose.position.y = 15;
    initialState.robotPose = initPose;
    Graph simulateTree;
    simulateTree.addNode(initialState,-1);

    nav_msgs::OccupancyGrid testing_map;
    testing_map.info.resolution = 0.5;
    testing_map.info.height = 100;
    testing_map.info.width = 100;

    std::cout << testing_map.info.width << std::endl;
    std::vector<int8_t> testMap;
    std::vector<int8_t> obsMap;
    for (int j = 0; j < 10000; j++){
        testMap.push_back(-1);
        obsMap.push_back((double) j / 100);
    }
    //std::cout << "check: " << (int) testMap[0] << std::endl;
    //testing_map.d
    testing_map.data = testMap;
    State initial_state;
    initial_state.map = testing_map;
    initial_state.robotPose = initPose;
    initial_state.robotPose.orientation.w = 1;

    //forwardSimulate(&initial_state,0);


    auto start = std::chrono::high_resolution_clock::now();


    //TODO::
    // Additional functionality:
    //  Need to complete 'Simulate' framework
    //rayCast(std::vector<int8_t> beliefMap, float theta, int mapX, int mapY, int map_limit_x, int map_limit_y,double map_resolution)
    //std::vector<int8_t> ray_result =  rayCast(testMap,0,10,50,100,100,testing_map.info.resolution);
    //reward mappings:
    std::vector<int8_t> obs_container;
    int action  = 1;
    for (int j = 0; j<4; j++){
        forwardSimulate(&initial_state,1,obs_container);
    }
    for (int j = 0; j<500; j++){
       float q =  forwardSimulate(&initial_state,action,obs_container);
       //action = 1;
       if (j % 2 != 0){
            action = 1;
        }
       //std::cout <<initial_state.robotPose.position.x << " " << initial_state.robotPose.position.y << std::endl;
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << duration.count() << std::endl;
    std::cout << initial_state.robotPose.position.x << std::endl;

    return 0;
}


