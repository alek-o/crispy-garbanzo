#include "..\include\WCGraphGenerator.h"
#include "../include/Graph.h"

#include <ctime>
#include <cstdlib>


WCGraphGenerator::WCGraphGenerator()
    : GraphGenerator()
{
    this->nodeCount = 10;
    this->directed = false;
    this->weighted = true;
}

//------------------------------------------------------------------------------------------------

WCGraphGenerator::WCGraphGenerator(int nodeCount) 
    : GraphGenerator(nodeCount)
{
    this->nodeCount = nodeCount;
    this->directed = false;
    this->weighted = true;
}

//------------------------------------------------------------------------------------------------

WCGraphGenerator::WCGraphGenerator(int nodeCount, int minHamiltonianCycles, int edgeCount)
    : GraphGenerator(nodeCount, false, true, minHamiltonianCycles, edgeCount)
{
    this->nodeCount = nodeCount;
    this->minHamiltonianCycles = minHamiltonianCycles;
    this->edgeCount = edgeCount;
}

//------------------------------------------------------------------------------------------------

void WCGraphGenerator::createMoreCycles(Graph &g)
{
    std::vector<bool> available(g.getNodeCount(), false);
    for (int node : g.getNodes())
    {
        if (g.getAdjacentNodes(node).size() < g.getNodeCount()-1)
            available[node] = true;
    }

    for (int i=0; (i<g.getNodeCount() && g.backtrackingHamiltonianCycles().size() < minHamiltonianCycles); i++)
    {
        int randomNode = rand() % g.getNodeCount();
        if (available[randomNode])
        {
            for (int j=0; j<g.getNodeCount(); j++)
            {
                if (randomNode==j)
                    continue;
                
                if (g.getWeight(randomNode, j) == -1)
                {
                    int weight = rand() % maxWeight + 1;
                    g.addEdge(randomNode, j, weight);
                    break;
                }
            }
        }    
    }
}

Graph WCGraphGenerator::generate()
{
    Graph g;
    
    srand(time(0));

    // Step 1: Create a Hamiltonian cycle
    for (int i = 0; i < nodeCount; ++i) {
        int next = (i + 1) % nodeCount; // wraps around to form a cycle
        int weight = rand() % maxWeight + 1;
        g.addEdge(i, next, weight);
    }

    // Step 2: Add extra edges to introduce additional cycles
    int extraEdges = nodeCount / 2; // add extra edges to create more cycles
    for (int i = 0; i < extraEdges; ++i) {
        int u = rand() % nodeCount;
        int v = rand() % nodeCount;
        if (u != v) {
            int weight = rand() % maxWeight + 1;
            g.addEdge(u, v, weight);
        }
    }

    return g;
}