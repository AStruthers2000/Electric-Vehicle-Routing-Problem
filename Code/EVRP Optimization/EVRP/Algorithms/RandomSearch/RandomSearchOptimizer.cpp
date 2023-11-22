#include "RandomSearchOptimizer.h"

#include "../../HelperFunctions.h"
#include "../../SolutionSet.h"

/**
 * \brief Generate #NUM_GENERATIONS * #SOLUTIONS_PER_GENERATION random solutions, saving the best from each generation.
 * We use random generation to generate #SOLUTIONS_PER_GENERATION purely random solutions. We save the best one, and
 * do this #NUM_GENERATIONS times. By the end, we will have #NUM_GENERATIONS "good" solutions. This could be used as
 * a good seed for other algorithms that start with an initial population.
 * \param best_solution
 */
void RandomSearchOptimizer::Optimize(solution &best_solution)
{
	auto* best_solutions = new SolutionSet();

	for (int i = 0; i < NUM_GENERATIONS; i++)
	{
		auto* generation_solutions = new SolutionSet();
		
		for (int j = 0; j < SOLUTIONS_PER_GENERATION; j++)
		{
			vector<Node> tour = problem_data->GenerateRandomTour();
			generation_solutions->AddSolutionToSet({tour, vehicle->SimulateDrive(tour)});
		}
		best_solutions->AddSolutionToSet(generation_solutions->GetBestSolution());
	}

	found_tours = new SolutionSet(best_solutions);
	best_solution = best_solutions->GetBestSolution();
	


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
