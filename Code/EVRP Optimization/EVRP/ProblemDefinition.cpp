#include "ProblemDefinition.h"

#include <random>

vector<Node> ProblemDefinition::GenerateRandomTour() const
{
    random_device rd;
    mt19937 rng(rd());
    vector<Node> shuffled = customer_nodes;
    shuffle(shuffled.begin(), shuffled.end(), rng);
    return shuffled;
}

