#pragma once

using namespace std;

#include <vector>
#include <string>
#include "node.hpp"
#include "edge.hpp"
#include "uility.hpp"

class Graph {
private:
    vector<Node> nodes;
    vector<Edge> edges;
public:
    // Adjacency list: node ID -> list of adjacent nodes
    vector<vector<AdjEntry>> adjList;

    // --- Node operations ---
    void addNode(const Node& node);
    void setNodeRanks(const std::vector<int>& rankOrder);
    Node getNodeById(int id) const;
    vector<Node> getAllNodes() const;
    int getNodeCount() const;
    vector<int> getNeighbors(int nodeId) const;
    vector<pair<int, int>> getAllNeighborPairs(const vector<int>& nodeIds) const;
    vector<Edge> contractNode(Node node);

    // --- Edge operations ---
    void addEdge(const Edge& edge);
    Edge createNewEdge(int src, int trg, int cost, int id = -1, bool shortcut = false, vector<Edge> bridgingEdges = {});
    Edge getEdgeById(const int& edgeId) const;
    vector<Edge> getEdgesByIds(const vector<int>& edgeIds) const;
    vector<Edge> getAllEdges() const ;
    int getEdgeCount() const;
    int getReverseEdgeId(int edgeId) const;
    void addShortcuts(const vector<Edge>& shortcuts);

    // --- Graph utilities ---
    void buildAdjList();  
    void printGraph() const;
};
