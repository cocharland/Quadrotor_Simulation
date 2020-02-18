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
#include "generateObservation.h"


bool DEBUG = false;

struct State {
    geometry_msgs::Pose robotPose;
    nav_msgs::OccupancyGrid map; //unseen == -1; 0-100
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
    int removeEdge(int fromNode, int toNode){
        //goto node
        bool found = false;
        for (int j = 0; j < adjacencyList.size(); j++){
            std::vector<int> tmp_container = adjacencyList[j];
            if (tmp_container[0] == fromNode){
                //were at the right point
                for(int k = 1; k < tmp_container.size(); k++){
                    if (tmp_container.at(k)==toNode){
                        //Now we have the link
                        std::iter_swap(adjacencyList.at(j).begin()+k, adjacencyList.at(j).end()-1);
                        found = true;
                        adjacencyList.at(j).pop_back(); //remove the element from the end of the list
                        return 0;
                    }
                }
            }
        }
        return 2;
    }
    void node_numbering_fixup(int rootNode){
        //function takes the root node sets its number to 0; and makes all other nodes logical increments from it.
        std::vector<int> number_mapping;
        number_mapping.push_back(rootNode);
        getNode(rootNode)->nodeNum = 0;
        auto it = nodeList.begin();
        for (int j = 0; j < nodeList.size(); j++){
            if(nodeList.at(j).nodeNum == rootNode){
                break;
            }
            it++;
        }
        std::iter_swap(it,nodeList.begin()); //switch the root node so it is in location 0
        int currentNode = 1;
        for (int j = 1; j < nodeList.size(); j++){
            number_mapping.push_back(nodeList.at(j).nodeNum);
            nodeList.at(j).nodeNum = currentNode;
            currentNode++;
        }
        
        //now remap the adjacency lists
        for (int j = 0; j < adjacencyList.size(); j++){
            std::vector<int> tmp_container = adjacencyList.at(j);
            for (int k = 0; k < tmp_container.size(); k++){
                int nodeTochange = adjacencyList.at(j).at(k);
                std::cout << nodeTochange << std::endl;

            }
        }
    }
    
    int prune_tree(int rootNode){
        //this performs BFS to find dosconnected portions of tree and removes them
        for (int vertex = 0; vertex < nodeList.size(); vertex++){
            nodeList.at(vertex).color=0; //mark all the nodes as white
            nodeList.at(vertex).d = -1;
            nodeList.at(vertex).pi = -1; // mark as uninitilized
            if (nodeList.at(vertex).nodeNum == rootNode){
                nodeList.at(vertex).color = 1;
                nodeList.at(vertex).d = 0;
                nodeList.at(vertex).pi = -1;
            }
        }
        //now enter BFS
        std::queue<int> Node_Queue;
        Node_Queue.push(rootNode); //queue first node

        while (!Node_Queue.empty()){
            int u = Node_Queue.front();
            Node_Queue.pop();
            std::vector<int> adj = getAdjacentNodes(u);
            for(int v = 0; v < adj.size(); v++){
                if (getNode(adj.at(v))->color == 0){
                    getNode(adj.at(v))->color = 1; //set color to grey
                    getNode(adj.at(v))->d++;
                    getNode(adj.at(v))->pi = u;
                    Node_Queue.push(adj.at(v));
                }
            }
            getNode(u)->color = 2; //set color to black
            std::cout << "U: " << u << std::endl;
        }
        //now remove all the nodes that are not black
        for (int j = 0; j < nodeList.size(); j++){
            if (nodeList.at(j).color != 2){
                //remove the node and the adjacency list
                std::iter_swap(nodeList.begin()+j,nodeList.end()-1); //swap the node to the end
                nodeList.pop_back();
                std::iter_swap(adjacencyList.begin()+j,adjacencyList.end()-1);
                adjacencyList.pop_back();
                j--;
            }
            std::cout << "J: " << j << std::endl;
        }
        //now only connected graph remains; need to reorder nodes to stay ontop of numbering
        node_numbering_fixup(rootNode);
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
    input_state->mapX = mapX;
    input_state->mapY = mapY;

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

    //std::cout << "X Cell: " << mapX << "\nY Cell: "<< mapY <<  std::endl;
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
                break;
        default: std::cout << "Never should be here. Code: 2" << std::endl;
    }
    if (RPY.yaw > 2 * M_PI ){ //bound fixup
        RPY.yaw -= 2 * M_PI;
    } else if (RPY.yaw < 0){
        RPY.yaw += 2 * M_PI;
    }
    if(DEBUG){
        std::cout << RPY.yaw << std::endl;
    }
    //Now the robot is pointed in the correct direction, but need to move in pointed direction.
    geometry_msgs::Quaternion q_u;
    euler2quat(&q_u, yaw_change );
    u_t.position.x = std::cos(RPY.yaw)*dist;
    u_t.position.y = std::sin(RPY.yaw)*dist; //change in pose stored
    u_t.position.z = 0;
    u_t.orientation = q_u;
    mapX = (x - input_state->map.info.origin.position.x + u_t.position.x) / input_state->map.info.resolution;
    mapY = (y - input_state->map.info.origin.position.y + u_t.position.y) / input_state->map.info.resolution;

    //std::cout << u_t.orientation.w <<" : " << u_t.orientation.z << std::endl;
    //Build an observation:
    std::vector<int8_t> observation = generate_observation(input_state->map.data);
    //Now: reward mapping
    //rayCast(std::vector<int8_t> beliefMap, float theta, int mapX, int mapY, int map_limit_x, int map_limit_y,double map_resolution)
    int observation_reward = 0;
    std::vector<int8_t> ray_result = rayCast(input_state->map.data,RPY.yaw,mapX,mapY,input_state->map.info.width,input_state->map.info.height,input_state->map.info.resolution);
    for (int j = 0; j < ray_result.size(); j++){
        if (ray_result.at(j) != -1){
            //now we are in the seen area
            if(input_state->map.data.at(j) == -1){
                observation_reward++;
            }
        }
    }
    //impact reward
    //Need to map from the vector to a point in the belief map *after* the motion
    //Tomorrow::try to get it to nav around a bit

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
    return 0;
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
    std::vector<int8_t> obsMap;
    for (int j = 0; j < 10000; j++){
        testMap.push_back(2);
        obsMap.push_back((double) j / 100);
    }
    //std::cout << "check: " << (int) testMap[0] << std::endl;
    //testing_map.d
    testing_map.data = testMap;
    State initial_state;
    initial_state.map = testing_map;
    initial_state.robotPose = initPose;
    initial_state.robotPose.orientation.w = 1;

    forwardSimulate(&initial_state,0);
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<int8_t> testObs = generate_observation(obsMap);
    if (DEBUG){
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
                std::cout << duration.count() << std::endl;
                int sum = 0;
                for (int j = 0; j < testObs.size(); j++){
                    sum += testObs[j];
                }
        }
    //TODO::
    // Additional functionality:
    //  Need to complete 'Simulate' framework
    //rayCast(std::vector<int8_t> beliefMap, float theta, int mapX, int mapY, int map_limit_x, int map_limit_y,double map_resolution)
    std::vector<int8_t> ray_result =  rayCast(testMap,M_PI,10,10,100,100,testing_map.info.resolution);
    //reward mappings:


    return 0;
}


