#pragma once
#include "../GraphStructure.h"
constexpr int POPULATION_SIZE = 1000; /*!< Size of the population, aka how many solutions should each successive generation have*/
constexpr int MAX_GENERATIONS = 10000; /*!< Number of generations the evolution will take place over.*/
constexpr int TOURNAMENT_SIZE = 10; /*!< The number of candidate solutions chosen at random from the current population when doing tournament selection*/
constexpr float MUTATION_RATE = 0.2; /*!< The percent chance that each child will get mutated*/

class GeneticAlgorithmOptimizer
{
public:
	void Optimize(const EVRP_Data data, std::vector<int>& bestTour, float& bestDistance);

private:
	std::vector<int> GenerateRandomTour(const int customerStart, const int size);

	std::vector<int> TournamentSelection(const std::vector<std::vector<int>> population, const std::vector<float> distances) const;
	std::vector<int> Crossover(const std::vector<int> parentTour1, const std::vector<int> parentTour2) const;
	void Mutate(std::vector<int>& child);

	int RandomNumberGenerator(const int min, const int max) const;
	void ShuffleVector(std::vector<int>& container);
	void PrintTour(const std::vector<int> tour) const;
};

