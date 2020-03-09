//
// Created by cody on 2/23/20.
//

#ifndef QUADROTOR_SIM_GRAPH_H
#define QUADROTOR_SIM_GRAPH_H


#include <vector>
#include "Structures.h"
#include <queue>

class Graph {
private:
    std::vector<std::vector<int>> adjacencyList;
    std::vector<adjNode> nodeList;
public:
    int numNodes;
    Graph();
    int addNode(State nodeState, int action);
    adjNode * getNode(int nodeNum);
    void addEdge(int fromNode, int toNode);
    std::vector<int> getAdjacentNodes(int nodeNumber);
    int removeEdge(int fromNode, int toNode);
    void node_numbering_fixup(int rootNode);
    int prune_tree(int rootNode);
    int addObsNode(std::vector<int8_t>);


};


#endif //QUADROTOR_SIM_GRAPH_H
