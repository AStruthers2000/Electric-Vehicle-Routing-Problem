#include "RandomSearchOptimizer.h"
#include "../../Vehicle.h"
#include "../../HelperFunctions.h"

/**
 * \brief Generate #NUM_GENERATIONS * #SOLUTIONS_PER_GENERATION random solutions, saving the best from each generation.
 * We use random generation to generate #SOLUTIONS_PER_GENERATION purely random solutions. We save the best one, and
 * do this #NUM_GENERATIONS times. By the end, we will have #NUM_GENERATIONS "good" solutions. This could be used as
 * a good seed for other algorithms that start with an initial population.
 * \param bestTour The best complete tour through all the customer nodes 
 * \param bestDistance The distance of the best tour
 */
void RandomSearchOptimizer::Optimize(vector<int>& bestTour, float& bestDistance)
{
	const int tourSize = static_cast<int>(problem_data.nodes.size()) - problem_data.customerStartIndex;

	map<vector<int>, float> bestSolutions;

	for (int i = 0; i < NUM_GENERATIONS; i++)
	{
		vector<int> _bestTour;
		float _bestDistance = numeric_limits<float>::max();

		//PrintIfTheTimeIsRight("Random Search", i, NUM_GENERATIONS);

		for (int j = 0; j < SOLUTIONS_PER_GENERATION; j++)
		{
			vector<int> tour = HelperFunctions::GenerateRandomTour(problem_data.customerStartIndex, tourSize);
			const float tourDistance = vehicle->SimulateDrive(tour, false);

			if (tourDistance < _bestDistance)
			{
				_bestDistance = tourDistance;
				_bestTour = tour;
			}
		}

		bestSolutions.emplace(_bestTour, _bestDistance);
	}

	bestDistance = numeric_limits<float>::max();
	for (const auto &iter : bestSolutions)
	{
		found_tours.emplace_back(iter.first);
		if (iter.second < bestDistance)
		{
			bestTour = iter.first;
			bestDistance = iter.second;
		}
	}


	/*
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
	*/
}
