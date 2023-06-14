#include "GAOptimizer.h"
#include <set>

void GAOptimizer::Optimize(const EVRP_Data data, std::vector<int> &bestTour, int &bestFitness, double &bestDistance)
{
	std::vector<std::vector<int>> population;
	std::vector<int> fitnesses;
	std::vector<double> distances;

	//generate initial population and fitnesses
	for (int i = 0; i < POPULATION_SIZE; i++)
	{
		population.push_back(GenerateRandomTour(data.customerStartIndex, (data.nodes.size() - data.customerStartIndex)));
		int fitness;
		double distance;
		EvaluateFitness(data, population[i], fitness, distance, false);

		fitnesses.push_back(fitness);
		distances.push_back(distance);
	}

	//iterate for MAX_GENERATIONS generations
	for (int generation = 0; generation < MAX_GENERATIONS; generation++)
	{
		if ((generation+1) % 10 == 0)
		{
			system("cls");
		}
		std::cout << "Percent complete: " << (double(generation) / MAX_GENERATIONS) * 100.0 << "%" << std::endl;

		std::vector<std::vector<int>> newPopulation;
		std::vector<int> newFitnesses;
		std::vector<double> newDistances;

		for (int i = 0; i < POPULATION_SIZE; i++)
		{
			//select parents
			//perform crossover between parents
			//mutate child

			std::vector<int> parentTour1 = TournamentSelection(data.nodes, population, fitnesses, data.loadCapacity);
			std::vector<int> parentTour2 = TournamentSelection(data.nodes, population, fitnesses, data.loadCapacity);
			std::vector<int> childTour = Crossover(parentTour1, parentTour2);
			if (std::rand() <= MUTATION_RATE)
			{
				Mutate(childTour);
			}

			//add child to new population and calculate new fitness 
			newPopulation.push_back(childTour);

			int fitness;
			double distance;
			EvaluateFitness(data, childTour, fitness, distance);
			newFitnesses.push_back(fitness);
			newDistances.push_back(distance);

		}
		population = newPopulation;
		fitnesses = newFitnesses;
		distances = newDistances;
	}
	
	bestFitness = std::numeric_limits<int>::max();
	bestDistance = std::numeric_limits<double>::max();
	for (int i = 0; i < POPULATION_SIZE; i++)
	{
		std::vector<int> tour = population[i];
		int fitness = fitnesses[i];
		double distance = distances[i];

		if (fitness <= bestFitness && distance < bestDistance)
		{
			bestTour = tour;
			bestFitness = fitness;
			bestDistance = distance;
		}
	}

	std::cout << "Best tour: ";
	PrintTour(bestTour);
	std::cout << "Fitness calculations: " << std::endl;
	int numSubtours;
	double distance;
	EvaluateFitness(data, bestTour, numSubtours, distance, true);
}

std::vector<int> GAOptimizer::GenerateRandomTour(const int customerStart, const int size)
{
	std::vector<int> tour(size);
	for (int i = customerStart; i < customerStart + size; i++)
	{
		tour[i - customerStart] = i;
	}
	ShuffleVector(tour);
	return tour;
}

void GAOptimizer::EvaluateFitness(const EVRP_Data data, const std::vector<int> tour, int& numSubtours, double& distance, const bool verbose) 
{
	std::vector<int> paddedTour;
	//fitness = number of times we have to visit the depot in this tour
	//fitness should be as small as possible. it will generally be > 0
	int fitness = 0;

	paddedTour.push_back(0);

	int current_loadCapacity = data.loadCapacity;
	float current_batteryCapacity = data.fuelCapacity;
	int currentNodeIndex = 0;

	std::vector<int> trueTour = tour;
	trueTour.push_back(0);

	for (int i = 0; i < trueTour.size(); i++)
	{
		int nextNodeIndex = trueTour[i];
		float potentialDistance = Distance(data.nodes[currentNodeIndex], data.nodes[nextNodeIndex]);
		float routeBatteryCost = potentialDistance * data.fuelConsumptionRate;
		
		if (verbose)	std::cout << "Starting this route from node " << currentNodeIndex << " and attempting to travel to node " << nextNodeIndex << "\n\tThis route costs " << routeBatteryCost << std::endl;

		int nextChargerIndex;
		if (CanGetToNextCustomerSafely(data, data.nodes[currentNodeIndex], data.nodes[nextNodeIndex], current_batteryCapacity))
		{
			current_batteryCapacity -= routeBatteryCost;
			if (verbose)	std::cout << "Can get to node " << trueTour[i] << " safely. Remaining battery cost: " << current_batteryCapacity << std::endl;
			currentNodeIndex = nextNodeIndex;
		}
		else if (CanGetToNextChargerSafely(data, data.nodes[currentNodeIndex], current_batteryCapacity, nextChargerIndex))
		{
			current_batteryCapacity = data.fuelCapacity;
			if (verbose)	std::cout << "Had to detour to charger " << nextChargerIndex << std::endl;
			currentNodeIndex = nextChargerIndex;
			i--;
		}
		else
		{
			std::cout << "Can't reach charging station, will run out of battery. Failed route" << std::endl;
			fitness += 100000; //huge punishment for impossible routes
			break;
		}

		paddedTour.push_back(currentNodeIndex);
		
		//if (verbose) std::cout << "\n\n======================================\n\n" << std::endl;

		/*
		int demand = data.nodes[tour[i]].demand;
		if(verbose)	printf("We have %d remaining supply, and node %d has demand %d\n", current_loadCapacity, tour[i] + 1, demand);
		if (current_loadCapacity - demand < 0)
		{
			//we needed to visit the depot befor servicing this node
			fitness++; 
			current_loadCapacity = data.loadCapacity;
			paddedTour.push_back(0);
			if (verbose) printf("\tBefore servicing this node, we must go back to the depot. We have visited the depot %d times this tour\n", fitness);
		}
		current_loadCapacity -= demand;
		paddedTour.push_back(tour[i]);
		if (verbose) printf("\tAfter servicing this node, we now have %d remaining supply\n", current_loadCapacity);
		*/
	}

	//We return to the depot at the end of the tour, so we add one to the fitness
	fitness++;
	//paddedTour.push_back(0);
	if (verbose) printf("Returning to the depot at the end of the tour\n====================\n");
	if (verbose) PrintTour(paddedTour);

	//PrintTour(paddedTour);
	distance = CalculateTotalDistance(data.nodes, paddedTour, data.loadCapacity);
	numSubtours = fitness;
}

double GAOptimizer::CalculateTotalDistance(const std::vector<Node> nodes, const std::vector<int>& tour, const int capacity) const
{
	Node home = Node{ 0, 0, 0 };
	int current_capacity = capacity;
	double tot = 0;

	for (int i = 1; i < tour.size(); i++)
	{
		Node from;
		Node to;
		if (tour[i - 1] == -1)
		{
			from = home;
		}
		else
		{
			from = nodes[tour[i - 1]];
		}

		if (tour[i] == -1)
		{
			to = home;
		}
		else
		{
			to = nodes[tour[i]];
		}

		tot += Distance(from, to);
	}
	return tot;
}

int GAOptimizer::GetClosestChargingStationToNode(std::vector<Node> nodes, Node node) const
{
	float closest = std::numeric_limits<float>::max();
	int closestChargerIndex = -1;

	for (int i = 0; i < nodes.size(); i++)
	{
		if (nodes[i].isCharger && nodes[i].index != node.index)
		{
			float dist = Distance(node, nodes[i]);
			if (dist < closest)
			{
				closestChargerIndex = i;
				closest = dist;
			}
		}
	}
	return closestChargerIndex;
}

bool GAOptimizer::CanGetToNextCustomerSafely(EVRP_Data data, Node from, Node to, float batteryRemaining)
{
	int chargerIndex = GetClosestChargingStationToNode(data.nodes, to);
	
	if (chargerIndex == -1)
	{
		return false;
	}

	Node closestCharger = data.nodes[chargerIndex];
	if (batteryRemaining > BatteryCost(data, from, to) + BatteryCost(data, to, closestCharger))
	{
		return true;
	}
	
	return false;
}

bool GAOptimizer::CanGetToNextChargerSafely(EVRP_Data data, Node from, float batteryRemaining, int& chargerIndex)
{
	chargerIndex = GetClosestChargingStationToNode(data.nodes, from);

	if (chargerIndex == -1)
	{
		return false;
	}


	Node closest = data.nodes[chargerIndex];
	if (batteryRemaining > BatteryCost(data, from, closest))
	{
		return true;
	}
	return false;
}


double GAOptimizer::Distance(const Node& node1, const Node& node2) const
{
	double dist = hypot(node1.x - node2.x, node1.y - node2.y);
	return dist;
}

double GAOptimizer::BatteryCost(const EVRP_Data data, const Node node1, const Node node2) const
{
	return Distance(node1, node2) * data.fuelConsumptionRate;
}

std::vector<int> GAOptimizer::TournamentSelection(const std::vector<Node> nodes, const std::vector<std::vector<int>> population, const std::vector<int> fitnesses, const int capacity) const
{
	std::vector<int> bestTour;
	int bestFitness = std::numeric_limits<int>::max();
	double bestDistance = std::numeric_limits<double>::max();

	for (int i = 0; i < std::max(2, TOURNAMENT_SIZE); i++)
	{
		int index = RandomNumberGenerator(0, population.size() - 1);
		std::vector<int> tour = population[index];
		int fitness = fitnesses[index];
		double distance = CalculateTotalDistance(nodes, tour, capacity);
		if (fitness <= bestFitness && distance < bestDistance)
		{
			bestTour = tour;
			bestFitness = fitness;
			bestDistance = distance;
		}
	}
	return bestTour;
}

std::vector<int> GAOptimizer::Crossover(const std::vector<int> parentTour1, const std::vector<int> parentTour2) const
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
	for (int i = 0; i < unique_v.size()-1; i++)
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

void GAOptimizer::Mutate(std::vector<int>& child)
{
	int index1 = RandomNumberGenerator(0, child.size() - 1);
	int index2 = RandomNumberGenerator(0, child.size() - 1);
	std::swap(child[index1], child[index2]);
}

int GAOptimizer::RandomNumberGenerator(const int min, const int max) const
{
	std::random_device rd;
	std::mt19937 generator(rd());
	std::uniform_int_distribution<int> distr(min, max);
	return distr(generator);
}

void GAOptimizer::ShuffleVector(std::vector<int>& container)
{
	std::random_device rd;
	std::mt19937 generator(rd());
	std::shuffle(container.begin(), container.end(), generator);
}

void GAOptimizer::PrintTour(const std::vector<int> tour) const
{
	std::cout << "Tour: ";
	for (int i = 0; i < tour.size(); i++)
	{
		std::cout << tour[i] << " ";
	}
	std::cout << std::endl;
}

