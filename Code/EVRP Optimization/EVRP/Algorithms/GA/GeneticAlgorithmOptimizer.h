#pragma once
#include "../AlgorithmBase.h"
#include "../../GraphStructure.h"
#include <numeric>
#include <vector>

constexpr int POPULATION_SIZE = 200; /*!< Size of the population, aka how many solutions should each successive generation have*/
constexpr int MAX_GENERATIONS = 1000; /*!< Number of generations the evolution will take place over.*/
constexpr int TOURNAMENT_SIZE = 20; /*!< The number of candidate solutions chosen at random from the current population when doing tournament selection*/
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

	void SetSeedSolutions(vector<vector<int>> seed);
	void Optimize(vector<int>& bestTour, float& bestDistance) override;

private:
	vector<int> TournamentSelection(const vector<vector<int>> population, const vector<float> distances) const;
	vector<int> Crossover(const vector<int> parentTour1, const vector<int> parentTour2) const;
	void Mutate(vector<int>& child);

	vector<vector<int>> seed_solutions;
	bool has_seed_solutions = false;

	float CalculateAverageSolution(vector<float> distances) const
	{
		if(distances.empty()) return 0;

		auto const count = static_cast<float>(distances.size());
		const float result = accumulate(distances.begin(), distances.end(), 0.f) / count;
		return result;
	}

	float CalculateBestSolution(vector<float> distances) const
	{
		if(distances.empty()) return 0;
		const float result = *min_element(begin(distances), end(distances));
		return result;
	}
};

