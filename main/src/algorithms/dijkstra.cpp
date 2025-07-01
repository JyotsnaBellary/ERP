//Dijkstra Algorithm
#include "dijkstra.hpp"
#include <queue>
#include <limits>
#include <iostream>

using namespace std;

Dijkstra::Dijkstra(const Graph& graph) : graph(graph) {}

/**
 * @brief Computes the shortest paths for a list of neighbor pairs.
 * 
 * This function takes a vector of pairs representing source and destination nodes,
 * and computes the shortest paths between each pair using Dijkstra's algorithm.
 *
 * @param neighborPairs A vector of pairs where each pair contains source and destination node IDs
 * @return A vector of ShortestPathResult, each containing the total cost and the path as a vector of edges
 */
vector<ShortestPathResult> Dijkstra::getShortestPaths(const vector<pair<int, int>>& neighborPairs) {
    vector<ShortestPathResult> shortestPaths;

    // Iterate through each pair of source and destination nodes
    for (const auto& pair : neighborPairs) {
        int src = pair.first;
        int dst = pair.second;
        auto result = compute(src, dst);
        
        
        if (result.totalCost != INT_MAX) {  
            // If a path exists
            vector<Edge> edges = graph.getEdgesByIds(result.edgeIds);
            shortestPaths.push_back({result.totalCost, edges});
        } else {
            shortestPaths.push_back({INT_MAX, {}});
        }
    }
    
    return shortestPaths;
}

/**
 * @brief Computes the shortest path from src to dst using Dijkstra's algorithm.
 * 
 * @param src The source node ID
 * @param dst The destination node ID
 * @return DijkstraResult containing the total cost and the path as a vector of edges
 */
DijkstraResult Dijkstra::compute(int src, int dst) {
    size_t n = graph.getNodeCount();
    // Initialize distance and predecessor vectors
    vector<int> dist(n, INT_MAX);
    vector<int> prev(n, -1);
    vector<int> viaEdge(n, -1);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    dist[src] = 0;  // Distance to source is 0
    pq.push({0, src});  // Push source node with distance 0

    // Main loop of Dijkstra's algorithm
    while (!pq.empty()) {

        int currentDist = pq.top().first;
        int currentNode = pq.top().second;
        pq.pop();

        // Early exit if we reach the destination
        if (currentNode == dst) {
            break;
        }

        //avoid visited array and use this instead
        if (currentDist > dist[currentNode]) continue;


        // Iterate through all neighbors of the current node
        for (const auto& neighbor : graph.adjList[currentNode]) {
            int neighborNode = neighbor.neighbor;
            int edgeCost = neighbor.cost;
            int edgeId = neighbor.edgeId;
            
            // If the neighbor has not been visited and the distance can be improved
            if (currentDist + edgeCost < dist[neighborNode]) {
                dist[neighborNode] = currentDist + edgeCost;  // Update distance
                prev[neighborNode] = currentNode;  // Update predecessor
                viaEdge[neighborNode] = edgeId;  // Update via edge
                pq.push({dist[neighborNode], neighborNode});  // Push updated distance to priority queue
            }
        }
    }

    //using edge id's for to visualization purposes
    vector<int> edgepath = reconstructPathWithEdgeIds(src, dst, viaEdge, prev);
    vector<int> nodePath = reconstructEdgePath(src, dst, prev);
    
    // Return distance and path from src to dst
    return {dist[dst], nodePath, edgepath};
}

/**
 * @brief Reconstructs the path from src to dst using edge IDs.
 * 
 * @param src The source node ID
 * @param dst The destination node ID
 * @param viaEdge The vector containing edge IDs used to reach each node
 * @param prev The vector containing the previous node in the path for each node
 * @return A vector of edge IDs representing the path from src to dst
 */
vector<int> Dijkstra::reconstructPathWithEdgeIds(int src, int dst, const vector<int> viaEdge, const vector<int> prev) const {
    vector<int> edgePath;
    
    for (int at = dst; at != src; at = prev[at]) {
        // No edge found (shouldn't happen if path exists)
        if (viaEdge[at] == -1) break; 

        // Add the edge ID to the path
        edgePath.push_back(viaEdge[at]);
    }

    reverse(edgePath.begin(), edgePath.end());
    return edgePath;
}

/**
 * @brief Reconstructs the path from src to dst using node IDs.
 * 
 * @param src The source node ID
 * @param dst The destination node ID
 * @param prev The vector containing the previous node in the path for each node
 * @return A vector of node IDs representing the path from src to dst
 */
vector<int> Dijkstra::reconstructEdgePath(int src, int dst, const vector<int> prev) const {
    vector<int> path;
    for (int at = dst; at != -1; at = prev[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());  // Reverse the path to get it from src to dst
    return path;
}


