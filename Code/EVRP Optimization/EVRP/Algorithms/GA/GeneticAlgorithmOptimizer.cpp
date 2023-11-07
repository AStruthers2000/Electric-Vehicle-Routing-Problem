#include "GeneticAlgorithmOptimizer.h"
#include <set>
#include "../../Vehicle.h"
#include "../../HelperFunctions.h"

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
* @param bestTour An out parameter that will represent the best tour upon completion of this function
* @param bestDistance An out parameter for the true distance of the best tour 
*/
void GeneticAlgorithmOptimizer::Optimize(vector<int>& bestTour, float& bestDistance)
{
	//Vehicle class used to calculate the fitness of each route. Initialized with each Node, the vehicle's batter capacity, load capacity, and battery consumption rate
	
	vector<vector<int>> population;
	vector<float> tourDistances;

	//generate initial population and fitnesses
	for (int i = 0; i < POPULATION_SIZE; i++)
	{
		//Generate initial solutions, then calculate the fitnesses using the Vehicle.SimulateDrive()
		vector<int> initialTour = HelperFunctions::GenerateRandomTour(problem_data.customerStartIndex, (static_cast<int>(problem_data.nodes.size()) - problem_data.customerStartIndex));
		float tourDistance = vehicle->SimulateDrive(initialTour);

		//Add the initial solutions and initial distances (fitness of solution) to respective vectors
		population.push_back(initialTour);
		tourDistances.push_back(tourDistance);
	}

	//iterate for #MAX_GENERATIONS generations
	for (int generation = 0; generation < MAX_GENERATIONS; generation++)
	{
		//cout << "Currently calculating generation: " << generation << endl;
		//PrintIfTheTimeIsRight("Genetic Algorithm", generation, MAX_GENERATIONS);
		//if (generation % (MAX_GENERATIONS / 100) == 0) 
		//cout << "Currently calculating generation: " << generation << " which is " << (static_cast<float>(generation) / static_cast<float>(MAX_GENERATIONS)) * 100.f << "% of the way done" << endl;

		vector<vector<int>> newPopulation;
		vector<float> newDistances;

		for (int i = 0; i < POPULATION_SIZE; i++)
		{
			//select parents
			//perform crossover between parents
			//mutate child
			const vector<int> parentTour1 = TournamentSelection(population, tourDistances);
			const vector<int> parentTour2 = TournamentSelection(population, tourDistances);
			vector<int> childTour = Crossover(parentTour1, parentTour2);
			const float r = HelperFunctions::RandomNumberGenerator(0, 100);
			//const float r = static_cast<float>(rand()) / RAND_MAX;
			if (r <= MUTATION_RATE * 100.f)
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

		/*
		//display best fitness each generation
		float best_gen_distance = numeric_limits<float>::max();
		for (int i = 0; i < POPULATION_SIZE; i++)
		{
			
			float distance = tourDistances[i];
			if (distance < best_gen_distance)
			{
				//bestTour = tour;
				best_gen_distance = distance;
			}
		}
		cout << "Best distance on generation " << generation << ": " << best_gen_distance << endl;
		*/
	}

	//select the best tour after #MAX_GENERATIONS generations
	bestDistance = numeric_limits<float>::max();
	for (int i = 0; i < POPULATION_SIZE; i++)
	{
		vector<int> tour = population[i];

		float distance = tourDistances[i];
		if (distance < bestDistance)
		{
			bestTour = tour;
			bestDistance = distance;
		}
	}

	//cout << "Best tour: ";
	//HelperFunctions::PrintTour(bestTour);
	//cout << "The best tour has distance breakdown: " << vehicle->SimulateDrive(bestTour, true) << endl;
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
vector<int> GeneticAlgorithmOptimizer::TournamentSelection(const vector<vector<int>> population, const vector<float> distances) const
{
	vector<int> bestTour;
	double bestDistance = numeric_limits<double>::max();

	for (int i = 0; i < max(2, TOURNAMENT_SIZE); i++)
	{
		int index = HelperFunctions::RandomNumberGenerator(0, static_cast<int>(population.size()) - 1);

		vector<int> tour = population[index];
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
vector<int> GeneticAlgorithmOptimizer::Crossover(const vector<int> parentTour1, const vector<int> parentTour2) const
{
	// Create a child vector with the same size as the parents
	vector<int> child(parentTour1.size());

	// Copy a random subset of elements from parent1 to the child
	//int crossoverPoint = rand() % parentTour1.size();
	int crossoverPoint = HelperFunctions::RandomNumberGenerator(0, parentTour1.size());
	copy_n(parentTour1.begin(), crossoverPoint, child.begin());

	// Fill the remaining elements in the child with unique elements from parent2
	int childIndex = crossoverPoint;
	for (const int element : parentTour2)
	{
		// Check if the element is already present in the child
		if (find(child.begin(), child.end(), element) == child.end())
		{
			child[childIndex] = element;
			++childIndex;
		}
	}
	//this is to assert that the child doesn't contain any duplicates (i.e. the crossover algorithm didn't preserve uniqueness of the elements)
	set<int> unique_s(child.begin(), child.end());
	vector<int> unique_v(unique_s.begin(), unique_s.end());
	for (size_t i = 0; i < unique_v.size() - 1; i++)
	{
		if (unique_v[i] + 1 != unique_v[i + 1])
		{
			cout << "\n\n\nERROR IN CROSSOVER!!!!\n\n\n" << endl;
			HelperFunctions::PrintTour(child);
			HelperFunctions::PrintTour(unique_v);
			cout << "\n\n\n======================\n\n\n" << endl;
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
void GeneticAlgorithmOptimizer::Mutate(vector<int>& child)
{
	int index1 = HelperFunctions::RandomNumberGenerator(0, static_cast<int>(child.size()) - 1);
	int index2 = HelperFunctions::RandomNumberGenerator(0, static_cast<int>(child.size()) - 1);
	swap(child[index1], child[index2]);
}
