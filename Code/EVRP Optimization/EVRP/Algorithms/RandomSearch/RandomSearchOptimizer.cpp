#include "RandomSearchOptimizer.h"
#include "../../Vehicle.h"
#include "../../HelperFunctions.h"

void RandomSearchOptimizer::Optimize(const EVRP_Data data, vector<int>& bestTour, float& bestDistance)
{
	//Vehicle class used to calculate the fitness of each route. Initialized with each Node, the vehicle's batter capacity, load capacity, and battery consumption rate
	Vehicle *vehicle = new Vehicle(data.nodes, data.fuelCapacity, data.loadCapacity, data.fuelConsumptionRate);
	
	int tourSize = static_cast<int>(data.nodes.size()) - data.customerStartIndex;

	map<vector<int>, float> bestSolutions;

	for (int i = 0; i < NUM_BEST_SOLUTIONS; i++)
	{
		vector<int> _bestTour;
		float _bestDistance = numeric_limits<float>::max();

		for (int j = 0; j < SOLUTIONS_PER_GENERATION; j++)
		{
			vector<int> tour = HelperFunctions::GenerateRandomTour(data.customerStartIndex, tourSize);
			float tourDistance = vehicle->SimulateDrive(tour, false);

			if (tourDistance < _bestDistance)
			{
				_bestDistance = tourDistance;
				_bestTour = tour;
			}
		}

		bestSolutions.emplace(_bestTour, _bestDistance);
	}

	bestDistance = numeric_limits<float>::max();
	for (auto iter : bestSolutions)
	{
		if (iter.second < bestDistance)
		{
			bestTour = iter.first;
			bestDistance = iter.second;
		}
	}


	cout << "Best tour: ";
	HelperFunctions::PrintTour(bestTour);
	cout << "The best tour has distance breakdown: " << vehicle->SimulateDrive(bestTour, true) << endl;

	cout << "Number of \"best\" solutions in solution map: " << bestSolutions.size() << endl;
	for (auto iter : bestSolutions)
	{
		HelperFunctions::PrintTour(iter.first);
		cout << "\t with distance " << iter.second << endl;
		cout << "---------------------------------------" << endl;
	}
}