#ifndef WAGRAPHGENERATOR_H
#define WAGRAPHGENERATOR_H

#include "GraphGenerator.h"

class WAGraphGenerator : public GraphGenerator
{
public:
    WAGraphGenerator();

    WAGraphGenerator(int nodeCount);

    WAGraphGenerator(int nodeCount, /* bool directed, bool weighted, */ int minHamiltonianCycles, int edgeCount = -1);

    Graph generate() override;
};

#endif // WAGRAPHGENERATOR1_H