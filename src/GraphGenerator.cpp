#include "../include/GraphGenerator.h"
#include "../include/Graph.h"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <list>
#include <utility>
#include <stdlib.h>
#include <time.h>
#include <algorithm>

GraphGenerator::GraphGenerator()
{
	this->nodeCount = 10;
}

//------------------------------------------------------------------------------------------------

GraphGenerator::GraphGenerator(int nodeCount)
{
	this->nodeCount = nodeCount;
}

//------------------------------------------------------------------------------------------------

GraphGenerator::GraphGenerator(int nodeCount, bool directed, bool weighted, int minHamiltonianCycles, int edgeCount)
{
	this->nodeCount = nodeCount;
	// this->directed = directed;
	// this->weighted = weighted;
	// this->graphType = graphType;
	this->minHamiltonianCycles = minHamiltonianCycles;
	this->edgeCount = edgeCount;
}

//------------------------------------------------------------------------------------------------

// void GraphGenerator::setWeighted(bool w)
// {
// 	weighted = w;
// }

//------------------------------------------------------------------------------------------------

// void GraphGenerator::setDirected(bool d)
// {
// 	directed = d;
// }

//------------------------------------------------------------------------------------------------

void GraphGenerator::setNodeCount(int count)
{
	nodeCount = count;
}

//------------------------------------------------------------------------------------------------

// void GraphGenerator::setGraphType(GraphType gt)
// {
// 	graphType = gt;
// }

//------------------------------------------------------------------------------------------------

void GraphGenerator::setMaxWeight(int w)
{
	maxWeight = w;
}

//------------------------------------------------------------------------------------------------

int GraphGenerator::getMaxWeight() const
{
	return maxWeight;
}

//------------------------------------------------------------------------------------------------

void GraphGenerator::setMinHamiltonianCycles(int num)
{
	minHamiltonianCycles = num;
	// if (minHamiltonianCycles > nodeCount/2)
	// 	minHamiltonianCycles = nodeCount/2;
}

//------------------------------------------------------------------------------------------------

// template<typename T>
// void getParams() const
// {

// }

//------------------------------------------------------------------------------------------------

// Graph GraphGenerator::generate() 
// {
// 	switch (graphType)
// 	{
// 		case DAG:
// 			directed = true;
// 			return generateDag();
// 			break;
// 		case DCG:
// 			directed = true;
// 			return generateDcgGraph();
// 			break;
// 		default:
// 			// std::cerr << "Invalid graph type!" << std::endl;
// 			return Graph();
// 	}
// }

GraphGenerator::~GraphGenerator() 
{

}