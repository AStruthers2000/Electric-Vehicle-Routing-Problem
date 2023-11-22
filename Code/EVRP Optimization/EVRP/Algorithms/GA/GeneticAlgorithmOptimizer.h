#pragma once
#include "../AlgorithmBase.h"
class SolutionSet;

constexpr int POPULATION_SIZE = 200; /*!< Size of the population, aka how many solutions should each successive generation have*/
constexpr int MAX_GENERATIONS = 500; /*!< Number of generations the evolution will take place over.*/
constexpr int TOURNAMENT_SIZE = 20; /*!< The number of candidate solutions chosen at random from the current population when doing tournament selection*/
constexpr float MUTATION_RATE = 0.2f; /*!< The percent chance that each child will get mutated*/

class GeneticAlgorithmOptimizer : public AlgorithmBase
{
public:
	GeneticAlgorithmOptimizer(const ProblemDefinition *data) :
		AlgorithmBase("Genetic Algorithm", data)
	{
		vector<string> hyper_parameters;
        
		hyper_parameters.push_back(string("Population Size: ") + to_string(POPULATION_SIZE));
		hyper_parameters.push_back(string("Maximum Generations: ") + to_string(MAX_GENERATIONS));
		hyper_parameters.push_back(string("Tournament Size: ") + to_string(TOURNAMENT_SIZE));
		hyper_parameters.push_back(string("Mutation Rate: ") + to_string(MUTATION_RATE));

		SetHyperParameters(hyper_parameters);
	}

	void SetSeedSolutions(const SolutionSet* seed);
	void Optimize(solution &best_solution) override;

private:
	solution TournamentSelection(const SolutionSet *current_population) const;
	solution Crossover(const solution &parent_1, const solution &parent_2) const;
	void Mutate(solution &child);

	SolutionSet* seed_solutions;
	bool has_seed_solutions = false;

	/*
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
	*/
};

