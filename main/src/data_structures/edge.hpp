// data_structures/edges.hpp
#pragma once

struct Edge {
    int id; 
    int src;
    int trg;
    int cost = 1; // Default to 1 as per file format
    bool shortcut = false; // Indicates if this edge is a shortcut added later
    vector<Edge> bridgingEdges; // Edges that this shortcut bridges if shortcut = true
};
