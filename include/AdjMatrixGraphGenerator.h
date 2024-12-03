#ifndef ADJMATRIXGRAPHGENERATOR_H
#define ADJMATRIXGRAPHGENERATOR_H

#include "GraphGenerator.h"
#include <random>

class AdjMatrixGraphGenerator : public GraphGenerator
{
private:
    double probability;
    
public:
    AdjMatrixGraphGenerator();

    AdjMatrixGraphGenerator(int nodeCount);

    AdjMatrixGraphGenerator(int nodeCount, /* bool directed, bool weighted, */ int minHamiltonianCycles, int edgeCount = -1);

    AdjMatrixGraphGenerator(int n, double p, int weightMax);

    Graph generate() override;
};  

#endif //ADJMATRIXGRAPHGENERATOR_H

