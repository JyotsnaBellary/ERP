// utils/file_handler.hpp
#pragma once

#include <string>
#include "data_structures/graph.hpp"
#include <unordered_set>
#include <set>

using namespace std;

class FileHandler {
public:
    FileHandler() = default;  // No filename stored
    void readInput(const std::string& filepath, Graph& graph);
    void exportToCSV(const Graph& graph, const string& nodeFile, const string& edgeFile, const vector<int>& shortestPathEdgeIds) const;
    void runPythonVisualizer(const std::string& scriptPath) const;
};
