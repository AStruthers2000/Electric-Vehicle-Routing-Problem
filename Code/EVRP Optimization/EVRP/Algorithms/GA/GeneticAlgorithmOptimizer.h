#pragma once
#include "../AlgorithmBase.h"
#include "../../GraphStructure.h"
constexpr int POPULATION_SIZE = 100; /*!< Size of the population, aka how many solutions should each successive generation have*/
constexpr int MAX_GENERATIONS = 2000; /*!< Number of generations the evolution will take place over.*/
constexpr int TOURNAMENT_SIZE = 15; /*!< The number of candidate solutions chosen at random from the current population when doing tournament selection*/
constexpr float MUTATION_RATE = 0.2f; /*!< The percent chance that each child will get mutated*/

class GeneticAlgorithmOptimizer : public AlgorithmBase
{
public:
	GeneticAlgorithmOptimizer(const EVRP_Data &data) :
		AlgorithmBase("Genetic Algorithm", data)
	{
		vector<string> hyper_parameters;
        
		hyper_parameters.push_back(string("Population Size: ") + to_string(POPULATION_SIZE));
		hyper_parameters.push_back(string("Maximum Generations: ") + to_string(MAX_GENERATIONS));
		hyper_parameters.push_back(string("Tournament Size: ") + to_string(TOURNAMENT_SIZE));
		hyper_parameters.push_back(string("Mutation Rate: ") + to_string(MUTATION_RATE));

		SetHyperParameters(hyper_parameters);
	}
	
	void Optimize(vector<int>& bestTour, float& bestDistance) override;

private:
	vector<int> TournamentSelection(const vector<vector<int>> population, const vector<float> distances) const;
	vector<int> Crossover(const vector<int> parentTour1, const vector<int> parentTour2) const;
	void Mutate(vector<int>& child);
};

