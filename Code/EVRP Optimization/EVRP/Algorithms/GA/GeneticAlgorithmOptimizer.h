#pragma once
#include "../../GraphStructure.h"
constexpr int POPULATION_SIZE = 100; /*!< Size of the population, aka how many solutions should each successive generation have*/
constexpr int MAX_GENERATIONS = 500; /*!< Number of generations the evolution will take place over.*/
constexpr int TOURNAMENT_SIZE = 5; /*!< The number of candidate solutions chosen at random from the current population when doing tournament selection*/
constexpr float MUTATION_RATE = 0.2f; /*!< The percent chance that each child will get mutated*/

class GeneticAlgorithmOptimizer
{
public:
	void Optimize(const EVRP_Data data, vector<int>& bestTour, float& bestDistance);

private:
	vector<int> TournamentSelection(const vector<vector<int>> population, const vector<float> distances) const;
	vector<int> Crossover(const vector<int> parentTour1, const vector<int> parentTour2) const;
	void Mutate(vector<int>& child);
};

