// data_structures/nodes.hpp
#pragma once

struct Node {
    int id;
    double lat;
    double lon;
    int rank;

    // Getter for id
    int getId() const { return id; }
    // Setter for id
    void setId(int newId) { id = newId; }

    // Getter for rank
    int getRank() const { return rank; }
    // Setter for rank
    void setRank(int newRank) { rank = newRank; }
};
