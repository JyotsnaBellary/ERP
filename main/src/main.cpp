#include <iostream>
#include "data_structures/graph.hpp"
#include "utils/file_handler.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/contraction_heirarchy.hpp"

using namespace std;

int main() {
    string filepath = "../RoadNetworks/test.txt"; // from build directory
    Graph graph;
    FileHandler fh;
    fh.readInput(filepath, graph);
    Dijkstra dijkstra(graph);

    // Example usage of Dijkstra's algorithm
    auto result = dijkstra.compute(0, 7); // Compute shortest paths from node 0

    cout << "Shortest path cost from 0 to 7: " << result.totalCost << endl;
    cout << "Node Ids: ";

    for (int node : result.nodePath) {
        cout << node << " ";
    }
    cout << endl;

    //To visualize the shortest path
    fh.exportToCSV(graph, "nodes.csv", "edges.csv", result.edgeIds);

    //Example usage of Contraction Hierarchy
    ContractionHeirarchy ch(graph);
    ch.preprocessing(); 

    return 0;
}
