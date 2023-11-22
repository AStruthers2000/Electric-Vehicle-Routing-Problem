#include "GeneticAlgorithmOptimizer.h"

#include <cassert>
#include <set>
#include "../../Vehicle.h"
#include "../../HelperFunctions.h"
#include "../../SolutionSet.h"

void GeneticAlgorithmOptimizer::SetSeedSolutions(const SolutionSet* seed)
{
	seed_solutions = new SolutionSet(seed);
	has_seed_solutions = true;
}

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
* @param best_solution
*/
void GeneticAlgorithmOptimizer::Optimize(solution &best_solution)
{
	//Vehicle class used to calculate the fitness of each route. Initialized with each Node, the vehicle's batter capacity, load capacity, and battery consumption rate
	auto *current_generation = new SolutionSet();

	int seed_solution_count = 0;
	if(has_seed_solutions)
	{
		cout << "GA using seed solutions" << endl;
		seed_solution_count = seed_solutions->GetNumberOfSolutions();
		for(const auto &seed : seed_solutions->GetSolutionSet())
		{
			current_generation->AddSolutionToSet(seed);

			if(current_generation->GetNumberOfSolutions() >= POPULATION_SIZE) break;
		}
	}
	
	//generate initial population and fitnesses
	for (int i = 0; i < POPULATION_SIZE - seed_solution_count; i++)
	{
		//Generate initial solutions, then calculate the fitnesses using the Vehicle.SimulateDrive()
		vector<Node> initial_tour = problem_data->GenerateRandomTour();
		solution initial_solution = {initial_tour, vehicle->SimulateDrive(initial_tour)};

		//Add the initial solutions and initial distances (fitness of solution) to respective vectors
		current_generation->AddSolutionToSet(initial_solution);
	}

	
	assert(current_generation->GetNumberOfSolutions() == POPULATION_SIZE);

	
	cout << "Average fitness for first generation: " << current_generation->GetAverageDistance() << endl;
	cout << "Best fitness for first generation: " << current_generation->GetBestSolution().distance << endl;
	/*
	if(has_seed_solutions)
	{
		ofstream file;
		file.open(R"(.\EVRP\Output\Average.txt)", ios_base::app);
		file << CalculateAverageSolution(tourDistances) << ",";
		file << CalculateBestSolution(tourDistances) << ",";
		file << "\n";
		file.close();
	}
	*/

	//iterate for #MAX_GENERATIONS generations
	for (int generation = 0; generation < MAX_GENERATIONS; generation++)
	{
		cout << "=================================================" << endl;
		cout << "Currently calculating generation: " << generation << endl;
		//PrintIfTheTimeIsRight("Genetic Algorithm", generation, MAX_GENERATIONS);
		//if (generation % (MAX_GENERATIONS / 100) == 0) 
		//cout << "Currently calculating generation: " << generation << " which is " << (static_cast<float>(generation) / static_cast<float>(MAX_GENERATIONS)) * 100.f << "% of the way done" << endl;

		//vector<vector<int>> newPopulation;
		//vector<float> newDistances;

		auto *next_generation = new SolutionSet();

		for (int i = 0; i < POPULATION_SIZE; i++)
		{
			//select parents
			//perform crossover between parents
			//mutate child
			//const vector<int> parentTour1 = TournamentSelection(population, tourDistances);
			//const vector<int> parentTour2 = TournamentSelection(population, tourDistances);
			const solution parent_solution_1 = TournamentSelection(current_generation);
			const solution parent_solution_2 = TournamentSelection(current_generation);
			
			solution child = Crossover(parent_solution_1, parent_solution_2);
			const int r = HelperFunctions::RandomNumberGenerator(0, 100);
			if (r <= static_cast<int>(MUTATION_RATE * 100.f))
			{
				Mutate(child);
			}

			//add child to new population and calculate new fitness 
			child.distance = vehicle->SimulateDrive(child.tour);

			next_generation->AddSolutionToSet(child);
		}
		current_generation = next_generation;

		assert(current_generation->GetNumberOfSolutions() == POPULATION_SIZE);

		
		cout << "Average fitness for generation " << generation << ": " << current_generation->GetAverageDistance() << endl;
		cout << "Best fitness for generation: " << generation << ": " << current_generation->GetBestSolution().distance << endl;
		/*
		if(has_seed_solutions && generation % 25 == 0)
		{
			ofstream file;
			file.open(R"(.\EVRP\Output\Average.txt)", ios_base::app);
			file << CalculateAverageSolution(tourDistances) << ",";
			file << CalculateBestSolution(tourDistances) << ",";
			file << "\n";
			file.close();
		}
		*/
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
	best_solution = current_generation->GetBestSolution();

	//cout << "Best tour: ";
	//HelperFunctions::PrintTour(bestTour);
	//cout << "The best tour has distance breakdown: " << vehicle->SimulateDrive(bestTour, true) << endl;
}

/**
* Critical element of the Genetic Algorithm.
* 
* Tournament selection selects the best parent out of #TOURNAMENT_SIZE possible parents
* 
* @param current_population The entire population, which consists of a vector of vectors of ints. We are selecting our candidate solution from the list of all solutions
* 
* @return Returns the best solution (lowest true distance) out of max(2, #TOURNAMENT_SIZE) solutions
*/
solution GeneticAlgorithmOptimizer::TournamentSelection(const SolutionSet *current_population) const
{
	auto *tournament_solutions = new SolutionSet();
	
	for (int i = 0; i < max(2, TOURNAMENT_SIZE); i++)
	{
		solution s = current_population->GetRandomSolution();
		tournament_solutions->AddSolutionToSet(s);
	}
	return tournament_solutions->GetBestSolution();
}


/**
* Critical element of the Genetic Algorithm.
* 
* Crossover performs Single Point Crossover, where a random number of 
* elements are selected from the first parent, then the rest of the
* elements are filled by unique elements of the second parent
* 
* @param parent_1 The first parent solution we will perform crossover on
* @param parent_2 The second parent solution for the crossover algorithm
* 
* @return A unique element crossover of parent1 and parent2. This vector should contain an unmodified subset of parent1, with the remaining indices filled with unique elements from parent2
*/
solution GeneticAlgorithmOptimizer::Crossover(const solution &parent_1, const solution &parent_2) const
{
	// Create a child vector with the same size as the parents
	//vector<int> child(parentTour1.size());
	//solution child = {};
	vector<Node> child_tour(parent_1.tour.size());
	//child_tour.reserve(parent_1.tour.size());

	// Copy a random subset of elements from parent1 to the child
	//int crossoverPoint = rand() % parentTour1.size();
	const int crossover_point = HelperFunctions::RandomNumberGenerator(0, static_cast<int>(parent_1.tour.size()));
	copy_n(parent_1.tour.begin(), crossover_point, child_tour.begin());

	// Fill the remaining elements in the child with unique elements from parent2
	int child_index = crossover_point;
	for (const Node &element : parent_2.tour)
	{
		// Check if the element is already present in the child
		if (find(child_tour.begin(), child_tour.end(), element) == child_tour.end())
		{
			child_tour[child_index] = element;
			++child_index;
		}
	}
	/*
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
	}*/

	solution child = {child_tour};
	return child;
}

/**
* Critical element of the Genetic Algorithm.
* 
* Mutate performs a single node swap.This mutation only happens with a #MUTATION_RATE percent chance per child
* 
* @param child The solution that needs to be mutated
*/
void GeneticAlgorithmOptimizer::Mutate(solution &child)
{
	const int index1 = HelperFunctions::RandomNumberGenerator(0, static_cast<int>(child.tour.size()) - 1);
	const int index2 = HelperFunctions::RandomNumberGenerator(0, static_cast<int>(child.tour.size()) - 1);
	swap(child.tour[index1], child.tour[index2]);
}
