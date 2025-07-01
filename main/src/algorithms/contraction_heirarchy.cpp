#include "contraction_heirarchy.hpp"
#include <unordered_set>
#include "dijkstra.hpp"
#include <iostream>
#include <file_handler.hpp>

using namespace std;

ContractionHeirarchy::ContractionHeirarchy(Graph &graph) : graph(graph)
{
    // Initialize rankOrder, upwardEdges, downwardEdges based on the original graph
    rankOrder.resize(graph.getNodeCount(), 0);
    upwardEdges.resize(graph.getNodeCount());
    downwardEdges.resize(graph.getNodeCount());
}

/**
 * @brief Calculates the edge difference for a given node.
 *
 * This function computes the difference in the number of edges before and
 * after contracting a node.
 *
 * @param graph The original graph
 * @param nodeId The ID of the node to be contracted
 * @return A pair containing the edge difference and the list of shortcut edges
 */
pair<int, vector<Edge>> ContractionHeirarchy::calculateEdgeDifference(Graph graph, int nodeId) const
{
    Graph modifiedGraph = graph;
    vector<Edge> shortcuts;
    int shortcut = 0;

    // Initialising Dijkstra
    Dijkstra dijkstra(modifiedGraph);

    // step 1: get all neighbors of nodeId
    vector<int> neighbors = modifiedGraph.getNeighbors(nodeId);
    int numNeighbors = neighbors.size();

    // Not enough neighbors to calculate edge difference
    if (numNeighbors < 2)
    {
        return {-1, {}};
    }

    // step 2: get all shortest path between all neighbors
    vector<pair<int, int>> neighborPairs = modifiedGraph.getAllNeighborPairs(neighbors);
    vector<ShortestPathResult> shortestPaths = dijkstra.getShortestPaths(neighborPairs);

    // step 3: contract node
    modifiedGraph.contractNode(graph.getNodeById(nodeId));

    // step 4: Find all neighbor pairs where shortest path has been affected
    vector<ShortestPathResult> newShortestPaths = dijkstra.getShortestPaths(neighborPairs);

    // step 5: see if the cost of path increased for any pair of neighbors or if no shortest path exists
    for (size_t i = 0; i < neighborPairs.size(); ++i)
    {
        const auto &original = shortestPaths[i];
        const auto &updated = newShortestPaths[i];

        if (updated.cost > original.cost || updated.cost == INT_MAX)
        {
            // Path cost increased
            shortcut++;

            // If the cost is INT_MAX, it means no path exists and use the original cost
            int costToUse = (updated.cost == INT_MAX) ? original.cost : updated.cost;

            // To update: Add a function to create bidirectional edges in the Graph class.

            // Create a new edges (shortcut) between the two neighbors
            Edge newEdge = modifiedGraph.createNewEdge(
                neighborPairs[i].first,
                neighborPairs[i].second,
                costToUse,    // Use the computed cost
                -1,           // Let the function assign a new ID
                true,         // Is this edge a shortcut?
                original.path // Edge path (edge IDs that were bridged)
            );

            Edge newReverseEdge = modifiedGraph.createNewEdge(
                neighborPairs[i].second,
                neighborPairs[i].first,
                costToUse,    // Use the computed cost
                -1,           // Let the function assign a new ID
                true,         // Is this edge a shortcut?
                original.path // Edge path (edge IDs that were bridged)
            );

            // Store the original path edges that this shortcut bridges
            newEdge.bridgingEdges = original.path;
            newReverseEdge.bridgingEdges = original.path;


            shortcuts.push_back(newEdge);
            shortcuts.push_back(newReverseEdge);
        }
    }

    // step 6: subtract the number of neighbors from the number of shortcuts added
    int edgeDifference = shortcut - numNeighbors;

    // step 7: return the difference and the shortcuts added
    return {edgeDifference, shortcuts};
}

/**
 * @brief Computes the rank order of nodes for Contraction Hierarchy.
 *
 * This function assigns a rank to each node based on the order in which
 * they are contracted.
 */
void ContractionHeirarchy::computeRankOrder()
{
    // Track already contracted nodes
    unordered_set<int> contracted;
    int currentRank = 0;
    Graph modifiedGraph = graph;

    //Loop until all nodes have been contracted
    while (contracted.size() < modifiedGraph.getNodeCount())
    {
        int minED = INT_MAX;
        ContractCandidateNode bestCandidate;
        bool found = false;

        //Find the node with least edge difference 
        for (const auto &node : modifiedGraph.getAllNodes())
        {

            int nodeId = node.id;
            if (contracted.count(nodeId))
                continue;

            // Calculate edge difference and get corresponding shortcuts
            auto [ed, shortcuts] = calculateEdgeDifference(modifiedGraph, nodeId);

            //skip if edge difference is not minimum
            if (ed > minED)
            {
                continue;
            } 

            else if (ed < minED)
            {
                // Found a better candidate for contraction
                minED = ed;
                bestCandidate = {nodeId, ed, move(shortcuts)};
                found = true;
            }
        }

        // If no more nodes to contract
        if (!found)
            break; 

        int nodeId = bestCandidate.nodeId;
        contracted.insert(nodeId);
        
        //Assign Rank
        rankOrder[nodeId] = currentRank++;   

        // Contract the node                          
        modifiedGraph.contractNode(modifiedGraph.getNodeById(nodeId)); 

        // To Check: what edges are being bridged and what would be the cost of the shortcut.
        modifiedGraph.addShortcuts(bestCandidate.shortcuts);
        for (const auto &edge : bestCandidate.shortcuts)
        {
            // Store shortcuts added during contraction
            shortcuts.push_back(edge); 
        }
    }
}

/**
 * @brief Classifies edges into upward and downward edges based on node ranks.
 *
 * An edge (u → v) is considered:
 *   - Upward if rank(u) < rank(v)
 *   - Downward otherwise
 *
 * The function populates `upwardEdges` and `downwardEdges` vectors
 */
void ContractionHeirarchy::computeUpwardDownwardEdges()
{
    upwardEdges.resize(graph.getNodeCount());
    downwardEdges.resize(graph.getNodeCount());

    for (const auto &edge : graph.getAllEdges())
    {
        if (rankOrder[edge.src] < rankOrder[edge.trg])
        {
            upwardEdges[edge.src].push_back(edge.trg);
        }
        else
        {
            downwardEdges[edge.src].push_back(edge.trg);
        }
    }
}

/**
 * @brief Builds two separate graphs for upward and downward edges.
 *
 * Uses the node rank order to partition the original graph's edges:
 *   - Edges from lower-rank to higher-rank nodes go into the upward graph.
 *   - Remaining edges go into the downward graph.
 *
 * These subgraphs are used in bidirectional search during queries.
 */
void ContractionHeirarchy::buildUpwardAndDownwardGraphs()
{
    upwardGraph = Graph();
    downwardGraph= Graph();
    for (const auto &edge : graph.getAllEdges())
    {
        if (rankOrder[edge.src] < rankOrder[edge.trg])
        {
            upwardGraph.addEdge(edge);
        }
        else {
            downwardGraph.addEdge(edge);
        }

    }
}

/**
 * @brief Preprocessing steps for Contraction Hierarchy.
 *
 * This function performs the various preprocessing steps required
 * to prepare the graph for efficient querying.
 */
void ContractionHeirarchy::preprocessing()
{
    // Step 1: Compute the rank of each node
    computeRankOrder();

    // Step 2: Store shortcuts and the edges they bridge
    graph.addShortcuts(shortcuts);

    // Step 3: Set the rank order in the graph
    graph.setNodeRanks(rankOrder);


    // step 4: compute upward and downward edges for each node
    computeUpwardDownwardEdges();

    // step 5: build adjacency list for upward and downward edges
    cout << "Building upward and downward adjacency lists..." << endl;
    buildUpwardAndDownwardGraphs(); // This will build the graph for the upward and downward edges of original graph

    // Step 7: Export the upward and downward graphs to CSV files for visualization
    FileHandler fh;
    fh.exportToCSV(upwardGraph, "upward_nodes.csv", "upward_edges.csv", {});
    fh.exportToCSV(downwardGraph, "downward_nodes.csv", "downward_edges.csv", {});
    fh.runPythonVisualizer("../output_files/visualizations/plot_network.py");
}

/**
 * @brief Queries the shortest path between two nodes using Contraction Hierarchy.
 *
 * This function implements a bidirectional Dijkstra's algorithm using the
 * upward and downward graphs built during preprocessing.
 *
 * @param src Source node ID
 * @param trg Target node ID
 */
void ContractionHeirarchy::queryPath(int src, int trg)
{
    // Implement bidirectional Dijkstra's algorithm using upward and downward graph
    // 1. Initialize distances and priority queues for both directions
    // 2. Start from both source and target nodes
    // 3. Expand nodes in both directions until they meet
    // 4. Combine results to find the shortest path
    // Note: Make sure to only use the upward edges when expanding from the source and downward edges when expanding from the target.
}