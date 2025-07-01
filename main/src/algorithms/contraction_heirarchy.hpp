#pragma once

#include "../data_structures/graph.hpp"

using namespace std;

class ContractionHeirarchy
{
    private:
        // Reference to the original graph
        Graph& graph;  

        // Rank order of nodes
        vector<int> rankOrder;        
        
        // Shortcuts added during contraction
        vector<Edge> shortcuts; 
        
        // Vector to store upward edges for each node
        vector<vector<int>> upwardEdges;   

        // Vector to store the downward edges for each node
        vector<vector<int>> downwardEdges;

        // Graph containing only upward edges
        Graph upwardGraph;   

        // Graph containing only downward edges
        Graph downwardGraph;

    public:
        
        // Constructor that initializes the contraction hierarchy with a given graph
        ContractionHeirarchy(Graph& graph);

        pair<int, vector<Edge>> calculateEdgeDifference(Graph graph, int nodeId) const;
        void computeRankOrder();
        void computeUpwardDownwardEdges();
        void buildUpwardAndDownwardGraphs();
        void preprocessing();
        void queryPath(int src, int dst);

};