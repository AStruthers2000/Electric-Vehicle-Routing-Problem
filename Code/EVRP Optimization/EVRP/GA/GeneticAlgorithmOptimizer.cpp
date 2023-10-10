#include "GeneticAlgorithmOptimizer.h"
#include <set>
#include "../Vehicle.h"

/**
* Core of the Genetic Algorithm.
* This function instanciates a Vehicle, that will simulate driving each of the routes, 
* as well as vectors that will hold the current generation and their fitnesses.
* The function first creates a population of #POPULATION_SIZE by randomly generating valid 
* tours through each of the customer nodes and calculates the fitness of each. Then the code 
* iterates for #MAX_GENERATIONS iterations, performing Tournament Selection, Single Point Crossover,
* and Mutation to generate a new population of #POPULATION_SIZE. Fitnesses of each of the children 
* are calculated via the Vehicle class. At the end of the generations, the child with the lowest 
* fitness is returned. The tour with the lowest distance at the end of #MAX_GENERATIONS should 
* be the most optimal route through each of the customer nodes.
* 
* Each tour is represented by a vector of ints, where each int is the index of a customer node in 
* the vector of all nodes. We only consider solutions that include 1 of each customer node as "valid"
* due to the restrictions of the EVRP. Since there is no requirement to visit each of the charging stations
* or the depot if we don't have to, solutions take the form of the order in which the Vehicle should visit
* each customer node. The Vehicle class will take the proposed tour and calculate the true distance of that
* route through simulation, stopping at a charging station or the depot whenever the proposed route demands it
* (ran out of inventory or needs to recharge battery before getting stranded). 
* 
* The fitness of each solution is represented by the true distance of the route as simulated by the Vehicle class.
* We seek to minimize the true distance through a Genetic Algorithm approach. 
* 
* @param data This is a reference to the EVRP_Data struct that contains all information regarding the current EVRP
* @param bestTour An out parameter that will represent the best tour upon completion of this function
* @param bestDistance An out parameter for the true distance of the best tour 
*/
void GeneticAlgorithmOptimizer::Optimize(const EVRP_Data data, std::vector<int>& bestTour, float& bestDistance)
{
	//Vehicle class used to calculate the fitness of each route. Initialized with each Node, the vehicle's batter capacity, load capacity, and battery consumption rate
	Vehicle* vehicle = new Vehicle(data.nodes, data.fuelCapacity, data.loadCapacity, data.fuelConsumptionRate);

	std::vector<std::vector<int>> population;
	std::vector<float> tourDistances;

	//generate initial population and fitnesses
	for (int i = 0; i < POPULATION_SIZE; i++)
	{
		//Generate initial solutions, then calculate the fitnesses using the Vehicle.SimulateDrive()
		std::vector<int> initialTour = GenerateRandomTour(data.customerStartIndex, (data.nodes.size() - data.customerStartIndex));
		float tourDistance = vehicle->SimulateDrive(initialTour);

		//Add the initial solutions and initial distances (fitness of solution) to respective vectors
		population.push_back(initialTour);
		tourDistances.push_back(tourDistance);
	}

	//iterate for #MAX_GENERATIONS generations
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
			std::vector<int> parentTour1 = TournamentSelection(population, tourDistances);
			std::vector<int> parentTour2 = TournamentSelection(population, tourDistances);
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

	//select the best tour after #MAX_GENERATIONS generations
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

/**
* Critical element of the Genetic Algorithm.
* 
* Tournament selection selects the best parent out of #TOURNAMENT_SIZE possible parents
* 
* @param population The entire population, which consists of a vector of vectors of ints. We are selecting our candidate solution from the list of all solutions
* @param distances The distances associated with each solution in the population. We want to pass this so that we don't have to recalculate the fitness of each solution on the fly
* 
* @return Returns the best solution (lowest true distance) out of max(2, #TOURNAMENT_SIZE) solutions
*/
std::vector<int> GeneticAlgorithmOptimizer::TournamentSelection(const std::vector<std::vector<int>> population, const std::vector<float> distances) const
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

/**
* Critical element of the Genetic Algorithm.
* 
* Crossover performs Single Point Crossover, where a random number of 
* elements are selected from the first parent, then the rest of the
* elements are filled by unique elements of the second parent
* 
* @param parentTour1 The first parent solution we will perform crossover on
* @param parentTour2 The second parent solution for the crossover algorithm
* 
* @return A unique element crossover of parent1 and parent2. This vector should contain an unmodified subset of parent1, with the remaining indices filled with unique elements from parent2
*/
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
	//this is to assert that the child doesn't contain any duplicates (i.e. the crossover algorithm didn't preserve uniqueness of the elements)
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

/**
* Critical element of the Genetic Algorithm.
* 
* Mutate performs a single node swap.This mutation only happens with a #MUTATION_RATE percent chance per child
* 
* @param child The solution that needs to be mutated
*/
void GeneticAlgorithmOptimizer::Mutate(std::vector<int>& child)
{
	int index1 = RandomNumberGenerator(0, child.size() - 1);
	int index2 = RandomNumberGenerator(0, child.size() - 1);
	std::swap(child[index1], child[index2]);
}

/**
* Helper functions used in the Genetic Algorithm code
* 
* Pseudo-random number generator that implements the standard C++ Mersenne Twister algorithm
* 
* @param min The lower end of the range of values in the uniform distribution (inclusive)
* @param max The upper end of the range of values in the uniform distribution (inclusive)
* 
* @return A uniformly distributed integer between min (inclusive) and max (inclusive)
*/
int GeneticAlgorithmOptimizer::RandomNumberGenerator(const int min, const int max) const
{
	std::random_device rd;
	std::mt19937 generator(rd());
	std::uniform_int_distribution<int> distr(min, max);
	return distr(generator);
}

/**
* Helper functions used in the Genetic Algorithm code.
*
* Generates a random tour through all customer nodes, represented by index number.
*
* @param customerStart The index representing the first customer node. When we read all nodes from the file, index 0 = depot node, and then there are some amount of charging nodes. We only care about generating indices for customer nodes
* @param size The number of total nodes in the list of all nodes. 
*
* @return A vector of node indices (ints) starting with customerStart and going to customerStart + size
*/
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

/**
* Helper functions used in the Genetic Algorithm code.
*
* Suffles a generic vector in place using the Mersenne Twister algorithm for the random number generator
*
* @param container The vector that needs to be shuffled
*
* @return A shuffled vector relative to the input container
*/
void GeneticAlgorithmOptimizer::ShuffleVector(std::vector<int>& container)
{
	std::random_device rd;
	std::mt19937 generator(rd());
	std::shuffle(container.begin(), container.end(), generator);
}

/**
* Helper functions used in the Genetic Algorithm code.
*
* Prints the tour in a human readable form, useful for debugging initial tours and final solution tour.
*
* @param tour The tour to be printed.
*/
void GeneticAlgorithmOptimizer::PrintTour(const std::vector<int> tour) const
{
	std::cout << "Tour: ";
	for (int i = 0; i < tour.size(); i++)
	{
		std::cout << tour[i] << " ";
	}
	std::cout << std::endl;
}

