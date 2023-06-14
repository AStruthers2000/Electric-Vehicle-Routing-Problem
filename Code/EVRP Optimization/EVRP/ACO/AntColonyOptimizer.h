#pragma once
#include "../GraphStructure.h"

class AntColonyOptimizer
{
public:
	AntColonyOptimizer(int nAnts, int nIterations, float PheromoneDecay, float Alpha, float Beta);

	void Optimize(std::vector<Node> Graph, int Capacity);

private:
	void ConstructSolution(std::vector<Node> Graph, int Capacity);
};

