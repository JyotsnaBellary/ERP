#include "utils/file_handler.hpp"
#include <fstream>
#include <iostream>
#include <vector>
#include <utility>
#include <cstdlib> 
#include <filesystem>
#include <set>
#include "data_structures/node.hpp"

using namespace std;

// Default cost for all edges
constexpr int EDGE_COST = 1;

/**
 * @brief Reads a graph from a file.
 * File format:
 *   n m
 *   node_id lat lon    (n times)
 *   src_id target_id   (m times)
 *
 * @param filepath Path to the input file
 * @param graph Reference to Graph object to populate
 */
void FileHandler::readInput(const string& filepath, Graph& graph) {
    ifstream file(filepath);
    if (!file.is_open()) {
        cerr << "Error: Unable to open file " << filepath << endl;
        return;
    }

    int n, m;
    file >> n >> m;

    // Read nodes
    for (int i = 0; i < n; ++i) {
        Node node;
        file >> node.id >> node.lat >> node.lon;
        graph.addNode(node);
    }

    // Read edges (bidirectional)
    int edgeId = 0; // Start custom edge ID counter
    for (int i = 0; i < m; ++i) {
        int src, trg;
        file >> src >> trg;

        Edge edge1 = graph.createNewEdge(src, trg, EDGE_COST, edgeId++);
        Edge edge2 = graph.createNewEdge(trg, src, EDGE_COST, edgeId++);

        // Check if the edge already exists in the graph
        bool edgeExists = false;
        for (const auto& edge : graph.getAllEdges()) {
            if ((edge.src == src && edge.trg == trg) || (edge.src == trg && edge.trg == src)) {
                edgeExists = true;
                break;
            }
        }

        if (!edgeExists) {
            graph.addEdge(edge1);
            graph.addEdge(edge2);
        }
    }

    graph.buildAdjList();
    file.close();
}

/**
 * @brief Exports the graph to CSV files.
 *
 * @param graph The graph to export
 * @param nodeFile Path to the output node CSV file
 * @param edgeFile Path to the output edge CSV file
 * @param shortestPathEdgeIds Vector of edge IDs representing the shortest path
 */
void FileHandler::exportToCSV(const Graph& graph,
                              const string& nodeFile,
                              const string& edgeFile,
                              const vector<int>& shortestPathEdgeIds) const {    
    // Export nodes
    ofstream nf(nodeFile);
    if (!nf) {
        cerr << "Failed to open " << nodeFile << "\n";
        return;
    }

    nf << "id,lat,lon\n";
    for (const auto& node : graph.getAllNodes()) {
        nf << node.id << "," << node.lat << "," << node.lon << "\n";
    }
    nf.close();

    // Create a set for fast lookup of shortest path edge IDs
    unordered_set<int> pathEdgeSet(shortestPathEdgeIds.begin(), shortestPathEdgeIds.end());

    // Export edges
    ofstream ef(edgeFile);
    if (!ef) {
        cerr << "Failed to open " << edgeFile << "\n";
        return;
    }

    ef << "id,source,target,on_shortest_path,shortcut\n";
    for (const auto& edge : graph.getAllEdges()) {

        //Highlight edges that are part of the shortest path
        bool highlight = pathEdgeSet.count(edge.id);

        // Check reverse direction
        int revId = graph.getReverseEdgeId(edge.id);
        if (!highlight && revId != -1) {
            highlight = pathEdgeSet.count(revId);
        }

        ef << edge.id << "," << edge.src << "," << edge.trg << "," << (highlight ? 1 : 0) << "," << (edge.shortcut ? 1 : 0) << "\n";
    }
    ef.close();

    cout << "CSV export complete: " << nodeFile << ", " << edgeFile << "\n";
}

/**
 * @brief Calls an external Python script to visualize the graph.
 *
 * @param scriptPath Path to the Python script
 */
void FileHandler::runPythonVisualizer(const string& scriptPath) const {
    string cmd = "python " + scriptPath;
    int result = system(cmd.c_str());
    if (result != 0) {
        cerr << "Failed to run Python script: " << scriptPath << "\n";
    } else {
        cout << "Visualization script executed successfully.\n";
    }
}