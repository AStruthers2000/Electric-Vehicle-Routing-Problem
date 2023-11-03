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
    const Node depot = problem_data.nodes[0];
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
    vector<vector<int>> optimal_subtours;
    optimal_subtours.reserve(subtours.size());

    for(const auto &subtour : subtours)
    {
        optimal_subtours.push_back(NEH_Calculation(subtour));
    }

    //now we have all of the subtours optimized as far as NEH can, so now we
    //combine them all into one complete tour and calculate the distance
    vector<int> tour;
    for(const auto &subtour : optimal_subtours)
    {
        for(const auto &node : subtour)
        {
            tour.push_back(node);
        }
    }
    bestTour = tour;
    bestDistance = vehicle->SimulateDrive(bestTour);
}

/**
 * \brief Find the closest node to the provided node while filtering the graph by nodes we have already visited.
 * \param customer_nodes All customer nodes in this problem instance
 * \param visited_nodes All nodes we have already visited in previous iterations
 * \param node We want to find the closest unvisited node to this node
 * \return We return the closest node to the node param
 */
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
    //we just call GetNearestNode on the unvisited node subset of the complete customer graph
    return GetNearestNode(unvisited_nodes, node);
}

/**
 * \brief Find the nearest node to any given node out of a set of possible nodes.
 * \param customer_nodes All customer nodes in which to search
 * \param node The node that we are searching around
 * \return The closest node to the input node param
 */
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

/**
 * \brief Use NEH concepts to find the best ordering of nodes in each subtour.
 * we need to figure out the optimal ordering of the nodes in the subtour to minimize the distance
 * in each subtour. We will use NEH concepts to do this.
 * \param subtour The unoptimized subtour generated by the nearest neighbor approach 
 * \return The optimal ordering of the nodes inside the given subtour
 */
vector<int> NEH_NearestNeighbor::NEH_Calculation(const vector<Node>& subtour) const
{
    //if there is only one node in the subtour, we want to return. it is already "ordered"
    if(subtour.size() == 1) return {subtour[0].index};

    vector<int> subtour_index;
    subtour_index.reserve(subtour.size());
    for(const auto &node : subtour)
    {
        subtour_index.push_back(node.index);
    }
    
    //first we need to figure out the optimal ordering of the first two elements in the subtour

    //start with l = 2
    int l = 2;
    vector<int> best_partial;
    vector<int> partial_subtour;// = {subtour_index.begin(), subtour_index.begin() + l};
    partial_subtour.push_back(subtour_index[0]);
    
    do
    {
        float best_dist = numeric_limits<float>::max();
        //we insert the next node into every possible point inside of the current subtour and calculate the distance
        for(size_t i = 0; i < partial_subtour.size() + 1; i++)
        {
            vector<int> temp_subtour = {partial_subtour.begin(), partial_subtour.end()};
            const auto index = temp_subtour.begin() + static_cast<long long>(i);
            temp_subtour.insert(index, subtour_index[l-1]);

            const float dist = vehicle->SimulateDrive(temp_subtour);
            if(dist < best_dist)
            {
                best_partial = temp_subtour;
                best_dist = dist;
            }
        }
        //copy the best subtour into partial_subtour
        partial_subtour = {best_partial.begin(), best_partial.end()};
        l++;
    } while(static_cast<size_t>(l) < subtour.size() + 1);

    return partial_subtour;
}
