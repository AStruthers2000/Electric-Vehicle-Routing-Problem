#include "SolutionSet.h"

#include "HelperFunctions.h"

solution SolutionSet::GetBestSolution() const
{
    if(!solution_set.empty())
    {
        solution best = *solution_set.begin();
        return best;
    }
    return solution{};
}

solution SolutionSet::GetRandomSolution() const
{
    if(!solution_set.empty())
    {
        const int random_index = HelperFunctions::RandomNumberGenerator(0, static_cast<int>(solution_set.size())-1);

        auto it = solution_set.begin();
        advance(it, random_index);
        return *it;
    }

    return solution{};
}

float SolutionSet::GetMinimumDistance() const
{
    return GetBestSolution().distance;
}

float SolutionSet::GetAverageDistance() const
{
    return sum_all_distances / static_cast<float>(num_solutions);
}

void SolutionSet::AddSolutionToSet(const solution& sol)
{
    solution_set.emplace(sol);
    sum_all_distances += sol.distance;
    num_solutions++;
}
