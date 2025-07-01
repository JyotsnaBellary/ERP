// data_structures/graph.cpp
#include "graph.hpp"
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <dijkstra.hpp>
 
using namespace std;

/**
 * @brief Adds a new node to the graph.
 *
 * This function appends the specified node to the graph's node list.
 *
 * @param node The Node object to be added
 */
void Graph::addNode(const Node& node) {
    nodes.push_back(node);
}

/**
 * @brief Retrieves all nodes.
 *
 * This function returns a vector containing all nodes present in the graph.
 *
 * @return A vector of Node objects
 */
vector<Node> Graph::getAllNodes() const {
    return nodes;
}

/**
 * @brief Retrieves the number of nodes in the graph.
 *
 * This function returns the total count of nodes present in the graph.
 *
 * @return The number of nodes
 */
int Graph::getNodeCount() const {
    return nodes.size();
}

/**
 * @brief Adds a new edge to the graph.
 *
 * This function appends the specified edge to the graph's edge list.
 *
 * @param edge The Edge object to be added
 */
void Graph::addEdge(const Edge& edge) {
    edges.push_back(edge);
}

/**
 * @brief Creates a new edge.
 *
 * This function initializes a new edge with the specified parameters.
 *
 * @param src The source node ID
 * @param trg The target node ID
 * @param cost The cost of the edge
 * @param id The ID of the edge (optional)
 * @param shortcut Indicates if the edge is a shortcut (default is false)
 * @param bridgingEdges A list of bridging edges (default is empty)
 * @return The newly created Edge object
 */
Edge Graph::createNewEdge(int src, int trg, int cost, int id, bool shortcut, vector<Edge> bridgingEdges) {
    Edge newEdge;
    newEdge.src = src;
    newEdge.trg = trg;
    newEdge.cost = cost;
    newEdge.id = id;
    newEdge.shortcut = shortcut; // Set the shortcut flag but default is false
    newEdge.bridgingEdges = bridgingEdges; // Store the bridging edges if any

    return newEdge;
}

/**
 * @brief Retrieves all edges.
 *
 * This function returns a vector containing all edges present in the graph.
 *
 * @return A vector of Edge objects
 */
vector<Edge> Graph::getAllEdges() const {
    return edges;
}

/**
 * @brief Retrieves the number of edges in the graph.
 *
 * This function returns the total count of edges present in the graph.
 *
 * @return The number of edges
 */
int Graph::getEdgeCount() const {
    return edges.size();
}

/**
 * @brief Builds the adjacency list representation of the graph.
 *
 * This function iterates through all edges and populates the adjacency list
 * for each node, allowing for efficient neighbor lookups.
 */
void Graph::buildAdjList() {
    size_t n = nodes.size(); 
    adjList.assign(n, {}); 
    
    // Clear existing adjacency list    
    for (auto& list : adjList) {
        list.clear();
    }
    
    for (const auto& edge : edges) {
        adjList[edge.src].push_back({edge.trg, edge.cost, edge.id});
    }
}

// For Debugging: Prints the graph structure
void Graph::printGraph() const {
    cout << "Nodes:\n";
    for (const auto& node : nodes) {
        cout << node.id << ": (" << node.lat << ", " << node.lon << ")\n";
    }

    cout << "\nEdges:\n";
    for (const auto& edge : edges) {
        cout << "ID " << edge.id << ": " << edge.src << " -- " << edge.trg << " (cost " << edge.cost << ")\n";
    }

    cout << "\nAdjacency List:\n";
    for (int id = 0; id < adjList.size(); ++id) {
        cout << id << ": ";
        for (const auto& [neighbor, cost, edgeId] : adjList[id]) {
            cout << neighbor << "(" << cost << ") ";
        }
        cout << "\n";
    }
}

/**
 * @brief Gets an edge by its ID.
 *
 * This function searches for an edge with the specified ID in the graph's edge list.
 * If found, it returns the edge; otherwise, it throws an exception.
 *
 * @param edgeId The ID of the edge to be retrieved
 * @return The Edge object with the specified ID
 */
Edge Graph::getEdgeById(const int& edgeId) const {
    for (const auto& edge : edges) {
        if (edge.id == edgeId) {
            return {edge};
        }
    }
}

/**
 * @brief Gets edges by their IDs.
 * 
 * This function retrieves all edges whose IDs are in the provided list.
 * 
 *  @param edgeIds A vector of edge IDs to be retrieved
 * @return A vector of Edge objects corresponding to the specified IDs
 */
vector<Edge> Graph::getEdgesByIds(const vector<int>& edgeIds) const {
    unordered_set<int> idSet(edgeIds.begin(), edgeIds.end());
    vector<Edge> selected;
    for (const auto& edge : edges) {
        if (idSet.count(edge.id)) {
            selected.push_back(edge);
        }
    }
    return selected;
}

/**
 * @brief Gets the reverse edge ID for a given edge ID.
 *
 * This function finds the reverse edge of the specified edge in the graph.
 * If the reverse edge is found, it returns its ID; otherwise, it returns -1.
 *
 * @param edgeId The ID of the edge whose reverse is to be found
 * @return The ID of the reverse edge, or -1 if not found
 */
int Graph::getReverseEdgeId(int edgeId) const {
    const Edge& e = getEdgeById(edgeId);
    for (const auto& revEdge : edges) {
        if (revEdge.src == e.trg && revEdge.trg == e.src) {
            return revEdge.id;
        }
    }
    return -1; // Not found
}

/**
 * @brief Gets a node by its ID.
 *
 * This function searches for a node with the specified ID in the graph's node list.
 * If found, it returns the node; otherwise, it throws an exception.
 *
 * @param id The ID of the node to be retrieved
 * @return The Node object with the specified ID
 */
Node Graph::getNodeById(int id) const {
    for (const auto& node : nodes) {
        if (node.id == id) {
            return node;
        }
    }
    throw runtime_error("Node with given ID not found");
}

/**
 * @brief Gets the neighbors of a node.
 *
 * This function retrieves all neighbors of a given node ID from the adjacency list.
 *
 * @param nodeId The ID of the node whose neighbors are to be retrieved
 * @return A vector of neighbor IDs
 */
vector<int> Graph::getNeighbors(int nodeId) const {
    vector<int> neighbors;
    for (const auto& adj : adjList[nodeId]) {
        neighbors.push_back(adj.neighbor); 
    }
    return neighbors;
}

/**
 * @brief Gets all pairs of neighbors from a list of node IDs.
 *
 * This function generates all unique pairs of nodes from the provided list
 * of node IDs. It is useful for creating pairs for further processing.
 *
 * @param nodeIds A vector of node IDs
 * @return A vector of pairs, where each pair contains two node IDs
 */
vector<pair<int, int>> Graph::getAllNeighborPairs(const vector<int>& nodeIds) const {
    vector<pair<int, int>> pairs;
    for (int i = 0; i < nodeIds.size(); ++i)
        for (int j = i + 1; j < nodeIds.size(); ++j)
            pairs.push_back(make_pair(nodeIds[i], nodeIds[j]));
    return pairs;
}


/**
 * @brief Contracts a node in the graph.
 *
 * This function removes all edges connected to the specified node and returns
 * the edges that were removed. The adjacency list is rebuilt after contraction.
 *
 * @param node The node to be contracted
 * @return A vector of Edge objects representing the edges that were removed
 */
vector<Edge> Graph::contractNode(Node node) {
    int nodeId = node.id;

    // Collect the edges that will be removed
    std::vector<Edge> contractedEdges;
    for (const auto& edge : edges) {
        if (edge.src == nodeId || edge.trg == nodeId) {
            contractedEdges.push_back(edge);
        }
    }

    // Remove those edges from the graph
    edges.erase(std::remove_if(edges.begin(), edges.end(),
        [nodeId](const Edge& e) {
            return e.src == nodeId || e.trg == nodeId;
        }), edges.end());

    // Rebuild adjacency list
    buildAdjList();  
    return contractedEdges;
}

/**
 * @brief Adds shortcuts to the graph.
 *
 * This function adds shortcut edges to the graph, which are edges that
 * connect nodes that are not directly connected in the original graph.
 *
 * @param shortcuts A vector of Edge objects representing the shortcuts to be added
 */
void Graph::addShortcuts(const vector<Edge>& shortcuts) {
    for (const Edge& edge : shortcuts) {
        Edge newEdge = edge;
        // Assign a new ID based on current size
        newEdge.id = edges.size(); 
        edges.push_back(newEdge);

        //add the reverse edge as well as its bidirectional
        Edge reverseEdge = newEdge;
        reverseEdge.src = newEdge.trg;
        reverseEdge.trg = newEdge.src;
        reverseEdge.id = edges.size(); // Assign a new ID for the reverse edge
        edges.push_back(reverseEdge);

        // cout << "Adding shortcut edge ID " << newEdge.id << ": " << newEdge.src << " -- " << newEdge.trg << " (cost " << newEdge.cost << ")\n";

        // Update the adjacency list (rewrite a function to do this. )
        adjList[newEdge.src].push_back({newEdge.trg, newEdge.cost, newEdge.id});
        adjList[newEdge.trg].push_back({newEdge.src, newEdge.cost, newEdge.id}); 
    }
}

/**
 * @brief Sets the rank order of nodes in the graph.
 *
 * This function assigns ranks to the nodes based on the provided rankOrder vector.
 *
 * @param rankOrder A vector containing the rank for each node
 */
void Graph::setNodeRanks(const std::vector<int>& rankOrder) {
    if (rankOrder.size() != nodes.size()) {
        throw invalid_argument("rankOrder size does not match number of nodes");
    }
    for (size_t i = 0; i < nodes.size(); ++i) {
        nodes[i].rank = rankOrder[i];
    }
}

// void Graph::exportToDot(const string& filename) const {
//     ofstream out(filename);
//     if (!out.is_open()) {
//         cerr << "Error: Cannot open file " << filename << " for writing.\n";
//         return;
//     }

//     out << "graph G {\n";

//     // Output nodes with their IDs as labels
//     for (const auto& node : nodes) {
//         out << "  " << node.id << " [label=\"" << node.id << "\"];\n";
//     }

//     // To avoid duplicate edges in undirected graph, keep track of printed edges
//     vector<vector<bool>> printed(nodes.size(), vector<bool>(nodes.size(), false));

//     for (const auto& [src, neighbors] : adjList) {
//         for (const auto& [trg, cost] : neighbors) {
//             if (!printed[src][trg] && !printed[trg][src]) {
//                 out << "  " << src << " -- " << trg << ";\n";
//                 printed[src][trg] = true;
//                 printed[trg][src] = true;
//             }
//         }
//     }

//     out << "}\n";
//     out.close();

//     cout << "Graph exported to " << filename << endl;
// }