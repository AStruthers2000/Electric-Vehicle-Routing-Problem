#pragma once
#include "../AlgorithmBase.h"

class NEH_NearestNeighbor : public AlgorithmBase
{
public:
    NEH_NearestNeighbor(const EVRP_Data& data) :
        AlgorithmBase("NEH with Nearest Neighbor Subtours", data)
    {
        vector<string> hyper_parameters;
        
        //SetHyperParameters(hyper_parameters);
    }

    void Optimize(vector<int>& bestTour, float& bestDistance) override;

private:
    typedef struct node_distances
    {
        Node me;
        map<Node, float> distance_map;
    } node_distances;

    Node GetNearestUnvisitedNode(const vector<Node> &customer_nodes, const vector<Node> &visited_nodes, const Node &node);
    Node GetNearestNode(const vector<Node> &customer_nodes, const Node &node);
};
