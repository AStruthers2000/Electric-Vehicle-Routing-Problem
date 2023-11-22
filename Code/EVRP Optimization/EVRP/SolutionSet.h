#pragma once
#include <set>

#include "ProblemDefinition.h"

enum SolutionType
{
    DefaultSolution = -1
};

struct solution
{
    vector<Node> tour;
    float distance;

    solution(vector<Node> t = {}, float dist = DefaultSolution) : tour(std::move(t)), distance(dist) {}
};

struct CompareSolution
{
    bool operator()(const solution &a, const solution &b) const
    {
        return a.distance < b.distance;
    }
};

class SolutionSet
{
public:
    SolutionSet()
    {
        solution_set.clear();
        sum_all_distances = 0.f;
        num_solutions = 0;
    }

    SolutionSet(const SolutionSet *other_solutions)
    {
        solution_set.clear();
        sum_all_distances = 0.f;
        num_solutions = 0;
        for(const auto& sol : other_solutions->solution_set)
        {
            AddSolutionToSet(sol);
        }
    }
    
    
    solution GetBestSolution() const;
    solution GetRandomSolution() const;
    float GetMinimumDistance() const;
    float GetAverageDistance() const;
    void AddSolutionToSet(const solution &sol);
    int GetNumberOfSolutions() const { return num_solutions; }
    multiset<solution, CompareSolution> GetSolutionSet() const { return solution_set; }

private:
    multiset<solution, CompareSolution> solution_set;
    float sum_all_distances = 0.f;
    int num_solutions = 0;
    
};
