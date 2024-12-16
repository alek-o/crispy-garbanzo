#ifndef GRAPH_H
#define GRAPH_H

#include "DataStructure.h"
#include <vector>
#include <unordered_map>
#include <string>

class Graph : public DataStructure
{
protected:
    size_t nodeCount;
    size_t edgeCount;
    bool directed;
    bool weighted;
    int global_max; // algorithm purposes
    std::vector<int> nodes;
    std::vector<std::vector<int>> hamCycles;
    std::unordered_map<int, std::vector<std::pair<int, int>>> graph;

    bool isHamiltonianSafe(int node, std::vector<int>& path, int pos);
    bool isCycleBackToStart(const std::vector<int> &path);
    void helperFindHamiltonianCycle(std::vector<int>& path, std::vector<bool>& visited, int pos);
    bool hasAllMandatoryEdges(const std::vector<int>& currentPath, const std::vector<std::pair<int, int>>& mandatoryEdges = {});
    std::vector<std::vector<int>> selectCyclesConsistingOfChosenEdges(const std::vector<std::vector<int>>& cycles, const std::vector<std::pair<int,int>>& mandatoryEdges);
    std::vector<std::vector<int>> chooseMinimalBottleneckCycles(const std::vector<std::vector<int>>& cycles);

public:
    Graph();
    Graph(bool d, bool w);

    int addNode(const int &node);
    int addEdge(const int &u, const int &v, int weight = 1);

    bool contains(const int node) const;
    size_t getNodeCount() const;
    size_t getEdgeCount() const;
    int getWeight(const int nodeA, const int nodeB) const;
    std::vector<int> getNodes();
    std::vector<int> getAdjacentNodes(const int node) const;
    std::vector<std::pair<int,int>> at(int node);

    int chooseRandomVertex();
    std::pair<int, int> chooseRandomEdge();

    std::vector<std::vector<int>> backtrackingHamiltonianCycles();
    std::vector<std::vector<int>> bruteForceHamiltonCycles();
    std::vector<std::vector<int>> bruteForceBTSP(std::vector<std::pair<int,int>> mandatoryEdges = {});
    std::vector<std::vector<int>> backtrackingBTSP(std::vector<std::pair<int,int>> mandatoryEdges = {});
    void backtrackingRecursiveExploration(int currentNode, std::vector<int> currentPath, std::vector<bool> visited, int maxCurrentPathWeight, std::vector<std::vector<int>>& results, const std::vector<std::pair<int,int>>& mandatoryEdges = {});
    std::vector<std::vector<int>> greedyBTSP(std::vector<std::pair<int,int>> mandatoryEdges = {});
    std::vector<std::vector<int>> antColonyBTSP(std::vector<std::pair<int,int>> mandatoryEdges = {});
    void saveToFile(std::string fileName) const;
    void loadFromFile(std::string fileName);

    bool isWeighted() const;
    bool isDirected() const;
    bool areNeighbours(const int nodeA, const int nodeB) const;
    bool isEmpty() const;

    void display() const override;
    void raw_display() const;

    void clear() override;
    
    ~Graph();
};

#endif // GRAPH_H
