//
// Created by cody on 2/23/20.
//

#include "Graph.h"


Graph::Graph(){
    numNodes = 0;
}
int Graph::addNode(State nodeState, int action){
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
int Graph::addObsNode(std::vector<int8_t> obsIn){
    adjNode nodeToAdd;
    nodeToAdd.nodeNum = numNodes;
    nodeToAdd.action = -1;
    nodeToAdd.observation = obsIn;
    nodeToAdd.N = 0;
    nodeToAdd.M = 0;
    nodeToAdd.Q = 0;
    nodeList.push_back(nodeToAdd);
    std::vector<int> list_into;
    list_into.push_back(numNodes); //Probably will need to fix implementation of node referencing
    adjacencyList.push_back(list_into);
    numNodes++;

    return nodeToAdd.nodeNum;
}
adjNode * Graph::getNode(int nodeNum){
    for (int i = 0;i<numNodes;i++){
        if (nodeList[i].nodeNum == nodeNum){
            return &nodeList[i];
        }
    }
}
void Graph::addEdge(int fromNode, int toNode){
    //need to get the current edge in the list.
    for (int ii = 0; ii < adjacencyList.size(); ii++){
        //Interate through the vectors
        std::vector<int> tmp_container = adjacencyList[ii];
        if (tmp_container[0] == fromNode){
            adjacencyList[ii].push_back(toNode); //add the connected node to the list
        }
    }
}

std::vector<int> Graph::getAdjacentNodes(int nodeNumber) {
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
int Graph::removeEdge(int fromNode, int toNode){
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
void Graph::node_numbering_fixup(int rootNode){
    //function takes the root node sets its number to 0; and makes all other nodes logical increments from it.
    std::vector<int> number_mapping;
    number_mapping.push_back(rootNode);
    std::cout << "ROOT: " << rootNode << std::endl;
    auto it = nodeList.begin();
    for (int j = 0; j < nodeList.size(); j++){
        if(nodeList.at(j).nodeNum == rootNode){
            break;
        }
        it++;
    }
    getNode(rootNode)->nodeNum = 0;
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

            //Not done yet?
        }
    }
}

int Graph::prune_tree(int rootNode){
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
            nodeList.pop_back(); //remove from the tree
            std::iter_swap(adjacencyList.begin()+j,adjacencyList.end()-1);
            adjacencyList.pop_back();
            numNodes--;
            j--;
        }
        std::cout << "J: " << j << std::endl;
    }
    //now only connected graph remains; need to reorder nodes to stay ontop of numbering
    node_numbering_fixup(rootNode);
}
