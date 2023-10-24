#include "NEH_NearestNeighbor.h"

#include "../../HelperFunctions.h"

/**
 * \brief Uses NEH concepts to try and minimize the distance of each subtour in the route.
 * We will first come up with all of the subtours through the graph with inventory capacity as our
 * only constraint, using the nearest neighbor approach. This will get us a set of subtours
 * through customer nodes, where at the end of the subtour, we must return to the depot.
 * We will then use NEH concepts to order the nodes in each subtour in the way that results
 * in the minimum "makespan" of the route, aka the distance of that route. 
 * \param bestTour The best complete tour through all the customer nodes 
 * \param bestDistance The distance of the best tour
 */
void NEH_NearestNeighbor::Optimize(vector<int>& bestTour, float& bestDistance)
{
    /*
     * First, we need to generate our tours using NN. We will get all of the customer nodes,
     * then select the closest node to the depot. We will then iterate, going to the next closest
     * node until we can't satisfy demand. This will form our nearest neighbor subtour set.
     */

    //depot node is always node 0
    Node depot = problem_data.nodes[0];
    vector<Node> customer_nodes;
    vector<Node> visited_nodes;
    
    for(auto node : problem_data.nodes)
    {
        if(node.demand > 0)
        {
            customer_nodes.push_back(node);
        }
    }

    const Node starting = GetNearestNode(customer_nodes, depot);
    vector<vector<Node>> subtours;
    
    while(visited_nodes.size() < customer_nodes.size())
    {
        int capacity = problem_data.loadCapacity;
        
        vector<Node> subtour;
        Node current = starting;
        capacity -= current.demand;
        subtour.push_back(current);
        visited_nodes.push_back(current);
        while(true)
        {
            const Node nearest = GetNearestUnvisitedNode(customer_nodes, visited_nodes, current);
            if(nearest.demand <= capacity)
            {
                capacity -= nearest.demand;
                current = nearest;
                subtour.push_back(current);
                visited_nodes.push_back(current);
                if(visited_nodes.size() == customer_nodes.size())
                {
                    break;
                }
            }
            else
            {
                break;
            }
        }
        subtours.push_back(subtour);
    }

    //now we have all of the subtours required for this route
    //we now need to implement NEH concepts to find the best ordering of each element in each subtour
    cout << "Subtours" << endl;
}

Node NEH_NearestNeighbor::GetNearestUnvisitedNode(const vector<Node>& customer_nodes, const vector<Node>& visited_nodes,
    const Node& node)
{
    vector<Node> unvisited_nodes;
    for(const auto customer : customer_nodes)
    {
        bool has_visited = false;
        for(const auto visited : visited_nodes)
        {
            if(customer.index == visited.index)
            {
                has_visited = true;
            }
        }
        if(!has_visited)
        {
            unvisited_nodes.push_back(customer);
        }
    }
    return GetNearestNode(unvisited_nodes, node);
}

Node NEH_NearestNeighbor::GetNearestNode(const vector<Node> &customer_nodes, const Node& node)
{
    Node closest_node = node;
    float smallest_distance = numeric_limits<float>::max();
    for(const auto other : customer_nodes)
    {
        const float node_distance = HelperFunctions::CalculateInterNodeDistance(node, other);
        if(node_distance < smallest_distance && node_distance > 1)
        {
            closest_node = other;
            smallest_distance = node_distance;
        }
    }
    return closest_node;
}
