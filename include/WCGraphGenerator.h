#ifndef WCGRAPHGENERATOR_H
#define WCGRAPHGENERATOR_H

#include "GraphGenerator.h"

class WCGraphGenerator : public GraphGenerator
{
private:

protected:

public:
    WCGraphGenerator();

    WCGraphGenerator(int nodeCount);

    WCGraphGenerator(int nodeCount, /* bool directed, bool weighted, */ int minHamiltonianCycles, int edgeCount = -1);

    void createMoreCycles(Graph &g);

    Graph generate() override;
};

#endif // WCGRAPHGENERATOR1_H