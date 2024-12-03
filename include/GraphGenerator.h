#ifndef GRAPHGENERATOR_H
#define GRAPHGENERATOR_H

#include "Generators.h"
#include "Graph.h"

class GraphGenerator : public Generators<Graph>
{
protected:
	size_t nodeCount; 
	bool directed;
	bool weighted;
	size_t edgeCount;
	unsigned int minHamiltonianCycles;
	unsigned int maxWeight;

public:
	GraphGenerator();
	GraphGenerator(int nodeCount);
	GraphGenerator(int nodeCount, bool directed, bool weighted, int minHamiltonianCycles = 1, int edgeCount = -1);

	void setWeighted(bool w = true);
	void setMaxWeight(int w);
	void setDirected(bool d = true);
	void setNodeCount(int count);
	void setMinHamiltonianCycles(int num = 1);

	int getMaxWeight() const;

	~GraphGenerator();
};

#endif // GRAPHGENERATOR_H