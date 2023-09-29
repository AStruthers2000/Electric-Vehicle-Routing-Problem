#include "GeneticAlgorithmOptimizer.h"
#include <set>
#include "../Vehicle.h"

/*
* Core of the Genetic Algorithm
* This function instanciates a Vehicle, that will simulate driving each of the routes, as well as vectors that will hold the current generation and their fitnesses
* The function first creates a population of POPULATION_SIZE by randomly generating valid tours through each of the customer nodes and calculates the fitness of each
* Then the code iterates for MAX_GENERATION iterations, performing Tournament Selection, Single Point Crossover, and Mutation to generate a new population of POPULATION_SIZE
* Fitnesses of each of the children are calculated via the Vehicle class
* At the end of the generations, the child with the lowest fitness is returned. 
* The tour with the lowest distance at the end of MAX_GENERATIONS should be the most optimal route through each of the customer nodes.
*/
void GeneticAlgorithmOptimizer::Optimize(const EVRP_Data data, std::vector<int>& bestTour, float& bestDistance)
{
	Vehicle* vehicle = new Vehicle(data.nodes, data.fuelCapacity, data.loadCapacity, data.fuelConsumptionRate);

	std::vector<std::vector<int>> population;
	std::vector<float> tourDistances;

	//generate initial population and fitnesses
	for (int i = 0; i < POPULATION_SIZE; i++)
	{
		std::vector<int> initialTour = GenerateRandomTour(data.customerStartIndex, (data.nodes.size() - data.customerStartIndex));
		float tourDistance = vehicle->SimulateDrive(initialTour);

		population.push_back(initialTour);
		tourDistances.push_back(tourDistance);
	}

	//iterate for MAX_GENERATIONS generations
	for (int generation = 0; generation < MAX_GENERATIONS; generation++)
	{
		std::cout << "Currently calculating generation: " << generation << std::endl;

		std::vector<std::vector<int>> newPopulation;
		std::vector<float> newDistances;

		for (int i = 0; i < POPULATION_SIZE; i++)
		{
			//select parents
			//perform crossover between parents
			//mutate child
			std::vector<int> parentTour1 = TournamentSelection(data.nodes, population, tourDistances, data.loadCapacity);
			std::vector<int> parentTour2 = TournamentSelection(data.nodes, population, tourDistances, data.loadCapacity);
			std::vector<int> childTour = Crossover(parentTour1, parentTour2);
			if (std::rand() <= MUTATION_RATE)
			{
				Mutate(childTour);
			}

			//add child to new population and calculate new fitness 
			float tourDistance = vehicle->SimulateDrive(childTour);

			newPopulation.push_back(childTour);
			newDistances.push_back(tourDistance);

		}
		population = newPopulation;
		tourDistances = newDistances;
	}

	//select the best tour after MAX_GENERATIONS generations
	bestDistance = std::numeric_limits<double>::max();
	for (int i = 0; i < POPULATION_SIZE; i++)
	{
		std::vector<int> tour = population[i];

		float distance = tourDistances[i];
		if (distance < bestDistance)
		{
			bestTour = tour;
			bestDistance = distance;
		}
	}

	std::cout << "Best tour: ";
	PrintTour(bestTour);
	std::cout << "The best tour has distance breakdown: " << vehicle->SimulateDrive(bestTour, true) << std::endl;
}

/*
* Critical elements of the Genetic Algorithm
* Tournament selection selects the best parent out of TOURNAMENT_SIZE possible parents
* Crossover performs Single Point Crossover, where a random number of elements are selected from the first parent, then the rest of the elements are filled by unique elements of the second parent
* Mutate performs a single node swap. This mutation only happens with a MUTATION_RATE percent chance per child
*/

std::vector<int> GeneticAlgorithmOptimizer::TournamentSelection(const std::vector<Node> nodes, const std::vector<std::vector<int>> population, const std::vector<float> distances, const int capacity) const
{
	std::vector<int> bestTour;
	double bestDistance = std::numeric_limits<double>::max();

	for (int i = 0; i < std::max(2, TOURNAMENT_SIZE); i++)
	{
		int index = RandomNumberGenerator(0, population.size() - 1);

		std::vector<int> tour = population[index];
		float distance = distances[index];

		if (distance < bestDistance)
		{
			bestTour = tour;
			bestDistance = distance;
		}
	}
	return bestTour;
}

std::vector<int> GeneticAlgorithmOptimizer::Crossover(const std::vector<int> parentTour1, const std::vector<int> parentTour2) const
{
	// Create a child vector with the same size as the parents
	std::vector<int> child(parentTour1.size());

	// Copy a random subset of elements from parent1 to the child
	int crossoverPoint = rand() % parentTour1.size();
	std::copy(parentTour1.begin(), parentTour1.begin() + crossoverPoint, child.begin());

	// Fill the remaining elements in the child with unique elements from parent2
	int childIndex = crossoverPoint;
	for (int i = 0; i < parentTour2.size(); ++i)
	{
		int element = parentTour2[i];
		// Check if the element is already present in the child
		if (std::find(child.begin(), child.end(), element) == child.end())
		{
			child[childIndex] = element;
			++childIndex;
		}
	}
	//this is to assert that the child doesn't contain any duplicates
	std::set<int> unique_s(child.begin(), child.end());
	std::vector<int> unique_v(unique_s.begin(), unique_s.end());
	for (int i = 0; i < unique_v.size() - 1; i++)
	{
		if (unique_v[i] + 1 != unique_v[i + 1])
		{
			std::cout << "\n\n\nERROR IN CROSSOVER!!!!\n\n\n" << std::endl;
			PrintTour(child);
			PrintTour(unique_v);
			std::cout << "\n\n\n======================\n\n\n" << std::endl;
		}
	}

	return child;
}

void GeneticAlgorithmOptimizer::Mutate(std::vector<int>& child)
{
	int index1 = RandomNumberGenerator(0, child.size() - 1);
	int index2 = RandomNumberGenerator(0, child.size() - 1);
	std::swap(child[index1], child[index2]);
}

/*
* Helper functions used in the Genetic Algorithm code
*/

int GeneticAlgorithmOptimizer::RandomNumberGenerator(const int min, const int max) const
{
	//Mersenne Twister prng 
	std::random_device rd;
	std::mt19937 generator(rd());
	std::uniform_int_distribution<int> distr(min, max);
	return distr(generator);
}

std::vector<int> GeneticAlgorithmOptimizer::GenerateRandomTour(const int customerStart, const int size)
{
	std::vector<int> tour(size);
	for (int i = customerStart; i < customerStart + size; i++)
	{
		tour[i - customerStart] = i;
	}
	ShuffleVector(tour);
	return tour;
}

void GeneticAlgorithmOptimizer::ShuffleVector(std::vector<int>& container)
{
	std::random_device rd;
	std::mt19937 generator(rd());
	std::shuffle(container.begin(), container.end(), generator);
}

void GeneticAlgorithmOptimizer::PrintTour(const std::vector<int> tour) const
{
	std::cout << "Tour: ";
	for (int i = 0; i < tour.size(); i++)
	{
		std::cout << tour[i] << " ";
	}
	std::cout << std::endl;
}

