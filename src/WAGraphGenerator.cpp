#include "../include/WAGraphGenerator.h"
#include "../include/Graph.h"

#include <ctime>
#include <cstdlib>


WAGraphGenerator::WAGraphGenerator()
    : GraphGenerator()
{
    // this->nodeCount = 10;
    this->directed = false;
    this->weighted = true;
}

//------------------------------------------------------------------------------------------------

WAGraphGenerator::WAGraphGenerator(int nodeCount) 
    : GraphGenerator(nodeCount)
{
    // this->nodeCount = nodeCount;
    this->directed = false;
    this->weighted = true;
}

//------------------------------------------------------------------------------------------------

WAGraphGenerator::WAGraphGenerator(int nodeCount, int minHamiltonianCycles, int edgeCount)
    : GraphGenerator(nodeCount, false, true, minHamiltonianCycles, edgeCount)
{

}

//------------------------------------------------------------------------------------------------

Graph WAGraphGenerator::generate()
{
	if (nodeCount < 2) {
		// std::cerr << "Number of nodes must be at least 2 to form a DAG.\n";
		// return Graph(directed, weighted);
		return Graph();
	}

	Graph g;
	int max_edges = nodeCount * (nodeCount - 1) / 2;
	int num_edges = max_edges / 2;

	srand(static_cast<unsigned>(time(0)));

	while (g.getEdgeCount() < num_edges)
	{
		int max_dest = nodeCount - 1;
		int src = rand() % (max_dest - 1);
		int dest = rand() % (max_dest - src) + (src + 1);
		int weight;
		// if (weighted)
			weight = rand() % maxWeight + 1;
		// else
		// 	weight = 1;
			
		g.addEdge(src, dest, weight);
	}

	return g;
}