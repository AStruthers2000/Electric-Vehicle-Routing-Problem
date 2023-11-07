#pragma once
#include "../AlgorithmBase.h"
#include "../../GraphStructure.h"

constexpr int SOLUTIONS_PER_GENERATION = 500; /*!< The number of solutions that will be randomly generated. Of n solutions, the top 1 will be saved */
constexpr int NUM_GENERATIONS = 1000; /*!< Number of "best" solutions desired, 1 from every "generation" */

class RandomSearchOptimizer : public AlgorithmBase
{
public:
    RandomSearchOptimizer(const EVRP_Data &data) :
        AlgorithmBase("Random Search", data)
    {
        vector<string> hyper_parameters;
        
        hyper_parameters.push_back(string("Solutions per Generation: ") + to_string(SOLUTIONS_PER_GENERATION));
        hyper_parameters.push_back(string("Number of Best Solutions: ") + to_string(NUM_GENERATIONS));

        SetHyperParameters(hyper_parameters);
    }
    
    void Optimize(vector<int>& bestTour, float& bestDistance) override;
};
