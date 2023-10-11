#pragma once
#include "../../GraphStructure.h"

constexpr int SOLUTIONS_PER_GENERATION = 10000; /*!< The number of solutions that will be randomly generated. Of n solutions, the top 1 will be saved */
constexpr int NUM_BEST_SOLUTIONS = 100; /*!< Number of "best" solutions desired, 1 from every "generation" */

class RandomSearchOptimizer
{
public:
	void Optimize(const EVRP_Data data, vector<int>& bestTour, float& bestDistance);
	
};
