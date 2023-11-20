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

	//get all customer nodes for ease of calculation in the subtour generation
	vector<Node> customer_nodes;
	for (const auto& node : problem_data.nodes)
	{
		if (node.demand > 0)
		{
			customer_nodes.push_back(node);
		}
	}

	/*
	* Beginning of Nearest Neighbor Subtour generation
	*/

	vector<vector<Node>> subtours;
	vector<Node> visited_nodes;

	while (visited_nodes.size() < customer_nodes.size())
	{
		//track the current subtour
		vector<Node> subtour;

		//start this subtour at the depot
		Node current = depot;

		//start the subtour with a full vehicle
		int capacity = problem_data.loadCapacity;

		//iterate until we need to return to the depot
		while (true)
		{
			//get the nearest unvisited node to the current node. the current node is the depot on the first iteration 
			const Node nearest = GetNearestUnvisitedNode(customer_nodes, visited_nodes, current);

			//if the nearest demand is too large for our current capacity, we must return to the depot aka this subtour is over
			if (nearest.demand > capacity) break;

			//set the current node to the nearest, symbolizing us "going" to that node
			current = nearest;

			//subtract the current demand from our capacity so that we can know when we have to end this subtour
			capacity -= current.demand;

			//"visit" the current node
			subtour.push_back(current);
			visited_nodes.push_back(current);

			//if we have visited all the customer nodes, this is the end of the current subtour
			if (visited_nodes.size() == customer_nodes.size())
			{
				break;
			}
		}
		//add current subtour to collection of all subtours
		subtours.push_back(subtour);
	}

	/*
	* End of Nearest Neighbor Subtour generation, but we should assert that we did it correctly
	*/

	//assert that we have the proper number of nodes in the full tour
	int node_count = 0;
	for (const auto& sub : subtours)
	{
		for (const auto& n : sub)
		{
			node_count++;
		}
	}
	//cout << "Sum of all nodes in subtours: " << node_count << endl;
	assert(node_count == static_cast<int>(customer_nodes.size()));

	//assert that each customer is visited once and only once. if this is not the case, we have broken one of the critical
	//constraints of the problem
	vector<Node> all_nodes_in_subtour;
	bool any_duplicate_node = false;
	for (const auto& sub : subtours)
	{
		for (const auto& n : sub)
		{
			for (const auto& temp : all_nodes_in_subtour)
			{
				if (temp.index == n.index)
				{
					cout << "Duplicate node detected: " << temp.index << endl;
					any_duplicate_node = true;
				}
			}
			all_nodes_in_subtour.push_back(n);
		}
	}
	assert(any_duplicate_node == false);



	//now we have all of the subtours required for this route
	//we now need to implement NEH concepts to find the best ordering of each element in each subtour
	vector<vector<int>> optimal_subtours;
	optimal_subtours.reserve(subtours.size());

	for (const auto& subtour : subtours)
	{
		optimal_subtours.push_back(NEH_Calculation(subtour));
	}

	//now we have all of the subtours optimized as far as NEH can, so now we
	//combine them all into one complete tour and calculate the distance
	vector<int> tour;
	for (const auto& subtour : optimal_subtours)
	{
		for (const auto& node : subtour)
		{
			tour.push_back(node);
		}
	}
	bestTour = tour;
	bestDistance = vehicle->SimulateDrive(bestTour);
	found_tours.emplace_back(bestTour);
	//cout << bestDistance << endl;
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
	for (const auto& customer : customer_nodes)
	{
		bool has_visited = false;
		for (const auto& visited : visited_nodes)
		{
			if (customer.index == visited.index)
			{
				has_visited = true;
			}
		}
		if (!has_visited)
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
Node NEH_NearestNeighbor::GetNearestNode(const vector<Node>& customer_nodes, const Node& node)
{
	Node closest_node = node;
	float smallest_distance = numeric_limits<float>::max();
	for (const auto& other : customer_nodes)
	{
		const float node_distance = HelperFunctions::CalculateInterNodeDistance(node, other);
		if (node_distance < smallest_distance && node_distance > 1)
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
	if (subtour.size() == 1) return { subtour[0].index };

	//take the provided Node subtour and construct a list of just these node's indices because
	//the vehicle SimulateDrive method takes an integer index-encoded tour
	vector<int> index_encoded_subtour;
	index_encoded_subtour.reserve(subtour.size());
	for (const auto& node : subtour)
	{
		index_encoded_subtour.push_back(node.index);
	}

	//start with L = 2
	int L = 2;

	//vector to track the best partial subtour (aka the partial subtour with the smallest "makespan")
	vector<int> best_partial;

	//the complete partial subtour, which starts with element 0 of the provided subtour
	vector<int> partial_subtour;
	partial_subtour.push_back(index_encoded_subtour[0]);

	//iterate until L = subtour size + 1
	do
	{
		//start with the current best distance equal to the largest float number
		float best_dist = numeric_limits<float>::max();

		//we insert the next node into every possible point inside of the current subtour and calculate the distance
		for (size_t i = 0; i < partial_subtour.size() + 1; i++)
		{
			//create a temporary subtour vector that we can insert an element into
			//we create this temporary subtour every iteration so that we don't need to remove the element
			//it'll just override each iteration. we could definitely just remove the inserted element after 
			//calculating, if this becomes an issue
			vector<int> temp_subtour = { partial_subtour.begin(), partial_subtour.end() };

			//create a vector iterator at index i of the temporary subtour vector
			const auto index = temp_subtour.begin() + static_cast<long long>(i);

			//insert element L (index L - 1) into the subtour
			temp_subtour.insert(index, index_encoded_subtour[L - 1]);

			//calculate the distance of the partial subtour (all constraints are implemented in Vehicle::SimulateDrive)
			const float dist = vehicle->SimulateDrive(temp_subtour);
			if (dist < best_dist)
			{
				best_partial = temp_subtour;
				best_dist = dist;
			}
		}
		//copy the best subtour into partial_subtour
		//the best partial subtour will have 1 additional element, so this operation increases the size of the partial_subtour by 1
		partial_subtour = { best_partial.begin(), best_partial.end() };

		//Increment L so that the next iteration will sort the next element with NEH methodologies
		L++;
	} while (static_cast<size_t>(L) < subtour.size() + 1);

	//return the subtour 
	return partial_subtour;
}
