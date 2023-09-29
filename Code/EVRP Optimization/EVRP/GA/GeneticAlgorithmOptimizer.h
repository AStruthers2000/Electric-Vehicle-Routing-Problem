#pragma once
#include "../GraphStructure.h"
constexpr int POPULATION_SIZE = 100;
constexpr int MAX_GENERATIONS = 1000;
constexpr int TOURNAMENT_SIZE = 10;
constexpr float MUTATION_RATE = 0.2;

class GeneticAlgorithmOptimizer
{
public:
	void Optimize(const EVRP_Data data, std::vector<int>& bestTour, float& bestDistance);

private:
	std::vector<int> GenerateRandomTour(const int customerStart, const int size);

	std::vector<int> TournamentSelection(const std::vector<Node> nodes, const std::vector<std::vector<int>> population, const std::vector<float> distances, const int capacity) const;
	std::vector<int> Crossover(const std::vector<int> parentTour1, const std::vector<int> parentTour2) const;
	void Mutate(std::vector<int>& child);

	int RandomNumberGenerator(const int min, const int max) const;
	void ShuffleVector(std::vector<int>& container);
	void PrintTour(const std::vector<int> tour) const;
};

