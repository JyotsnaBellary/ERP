#pragma once

#include "../data_structures/graph.hpp"

using namespace std;

struct DijkstraResult {
    int totalCost;
    std::vector<int> nodePath;
    std::vector<int> edgeIds; 
};

class Dijkstra {
    private:
    const Graph& graph;
    public:
        // Constructor: takes a reference to the graph
        Dijkstra(const Graph& graph);
        
        vector<ShortestPathResult> getShortestPaths(const vector<pair<int, int>>& neighborPairs);

        // Main query function
        DijkstraResult compute(int src, int dst);

        // Optionally expose distance and predecessor maps
        const vector<int>& getDistances() const;
        const vector<int>& getPredecessors() const;
        const vector<int>& getViaEdges() const;

        vector<int> reconstructEdgePath(int src, int dst, vector<int> prev) const;
        vector<int> reconstructPathWithEdgeIds(int src, int dst, vector<int> viaEdge, vector<int> prev) const;
};