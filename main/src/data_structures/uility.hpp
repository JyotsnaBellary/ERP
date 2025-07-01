#pragma once
#include <vector>
#include "edge.hpp"

using namespace std;

struct ContractCandidateNode {
            int nodeId;
            int edgeDiff;
            vector<Edge> shortcuts;
            
};

struct AdjEntry {
    int neighbor;
    int cost;
    int edgeId;
};

struct ShortestPathResult {
    int cost;
    vector<Edge> path;
};
