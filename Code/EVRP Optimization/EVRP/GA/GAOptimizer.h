#pragma once
#include "../GraphStructure.h"

constexpr int POPULATION_SIZE = 100;
constexpr int MAX_GENERATIONS = 1000;
constexpr int TOURNAMENT_SIZE = 10;
constexpr float MUTATION_RATE = 0.15;

class GAOptimizer
{
public:
	void Optimize(const EVRP_Data data, std::vector<int>& bestTour, int& bestFitness, double& bestDistance);

private:
	std::vector<int> GenerateRandomTour(const int customerStart, const int size);
	
	void EvaluateFitness(const EVRP_Data data, const std::vector<int> tour, int &numSubtours, double &distance, const bool verbose = false);
	double CalculateTotalDistance(const std::vector<Node> nodes, const std::vector<int>& tour, const int capacity) const;
	double Distance(const Node& node1, const Node& node2) const;
	double BatteryCost(const EVRP_Data data, const Node node1, const Node node2) const;
	
	std::vector<int> TournamentSelection(const std::vector<Node> nodes, const std::vector<std::vector<int>> population, const std::vector<int> fitnesses, const int capacity) const;
	std::vector<int> Crossover(const std::vector<int> parentTour1, const std::vector<int> parentTour2) const;
	void Mutate(std::vector<int>& child);

	int RandomNumberGenerator(const int min, const int max) const;
	void ShuffleVector(std::vector<int>& container);
	void PrintTour(const std::vector<int> tour) const;

	int GetClosestChargingStationToNode(std::vector<Node> chargers, Node node) const;
	bool CanGetToNextCustomerSafely(EVRP_Data data, Node from, Node to, float batteryRemaining);
	bool CanGetToNextChargerSafely(EVRP_Data, Node from, float batteryRemaining, int& chargerIndex);
};

