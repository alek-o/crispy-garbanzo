#include "..\include\AdjMatrixGraphGenerator.h"
#include "../include/Graph.h"

#include <ctime>
#include <cstdlib>
#include <vector>
#include <random>


AdjMatrixGraphGenerator::AdjMatrixGraphGenerator()
    : GraphGenerator()
{
    this->nodeCount = 10;
    this->directed = false;
    this->weighted = true;
}

//------------------------------------------------------------------------------------------------

AdjMatrixGraphGenerator::AdjMatrixGraphGenerator(int nodeCount) 
    : GraphGenerator(nodeCount)
{
    this->nodeCount = nodeCount;
    this->directed = false;
    this->weighted = true;
    this->maxWeight = 10;
    this->probability = 0.75;
}

//------------------------------------------------------------------------------------------------

AdjMatrixGraphGenerator::AdjMatrixGraphGenerator(int nodeCount, int minHamiltonianCycles, int edgeCount)
    : GraphGenerator(nodeCount, false, true, minHamiltonianCycles, edgeCount)
{
    this->nodeCount = nodeCount;
    this->minHamiltonianCycles = minHamiltonianCycles;
    this->edgeCount = edgeCount;
    this->maxWeight = 10;
    this->probability = 0.75;
}

AdjMatrixGraphGenerator::AdjMatrixGraphGenerator(int nodeCount, double probability, int weightMax)
{
    this->nodeCount = nodeCount;
    this->probability = probability;
    this->maxWeight = weightMax;
    this->maxWeight = 10;
    this->probability = 0.75;
}

//------------------------------------------------------------------------------------------------

Graph AdjMatrixGraphGenerator::generate()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> weightDis(1, maxWeight); 
    // int weightMin = 1;
    std::uniform_real_distribution<> dis(0.0, 1.0); 

    std::vector<std::vector<int>> adjMatrix(nodeCount, std::vector<int>(nodeCount, 0));

    for (int i = 0; i < nodeCount; ++i) 
    {
        for (int j = i + 1; j < nodeCount; ++j) 
        {
            if (dis(gen) < probability) {
                int weight = weightDis(gen);
                adjMatrix[i][j] = weight;
                adjMatrix[j][i] = weight;
            }
        }
    }

    Graph graph = Graph(false, true);
    for (int i = 0; i < nodeCount; ++i) {
        for (int j = i + 1; j < nodeCount; ++j) {
            if (adjMatrix[i][j] > 0) {
                graph.addEdge(i, j, adjMatrix[i][j]);
            }
        }
    }

    return graph;
};