#pragma once
#include "../AlgorithmBase.h"
#include <map>


class NEH_NearestNeighbor : public AlgorithmBase
{
public:
    NEH_NearestNeighbor(const ProblemDefinition* data) :
        AlgorithmBase("NEH with Nearest Neighbor Subtours", data)
    {
        vector<string> hyper_parameters;
        hyper_parameters.emplace_back("None");
        SetHyperParameters(hyper_parameters);
    }

    void Optimize(solution &best_solution) override;

private:
    typedef struct node_distances
    {
        Node me;
        map<Node, float> distance_map;
    } node_distances;

    static Node GetNearestUnvisitedNode(const vector<Node> &customer_nodes, const vector<Node> &visited_nodes, const Node &node);
    static Node GetNearestNode(const vector<Node> &customer_nodes, const Node &node);
    solution NEH_Calculation(const solution &subtour) const;
};
