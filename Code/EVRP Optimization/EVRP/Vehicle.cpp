#include "Vehicle.h"

#include <queue>
#include <unordered_map>

#include "HelperFunctions.h"

/**
* Fitness calculation for the provided tour.
* 
* This function simulates the drive through the desired tour of customer nodes. Remember, the "solution"
* is defined by the preferred order of visiting each customer node. This desired route doesn't take into 
* account any visits to the charging stations or the depot, so we need to artificially add those in.
* Since the EVRP gives us flexibility with when we visit these special nodes, and there is no requirement
* that we *must* visit these nodes, we don't include any visits to these nodes in the desired route. 
* 
* This means that, since we don't know the true route once we start implementing constraints like fuel and capacity,
* we must find out the "true" route. We do that in this function. I'm positive that there's probably some graph theory 
* or other math that I could do to make this function less messy and less confusing, but I don't have a better implementation
* at this time. 
* 
* @param desiredRoute The tour through just the customer nodes.
* @param verbose This is false by default, and will hide a lot of the print outputs. This should not be set to true unless you want to output one specific route. In general, the outputting adds a lot of time to the simulation execution
* 
* @return Returns the true distance that the desired route would actually traverse with the fuel and capacity constraints
*/
/*
float Vehicle::SimulateDrive(const vector<int>& desiredRoute, bool verbose)
{
	ResetVehicle();
	if(verbose)
	{
		cout << "Simulating drive of route: ";
		HelperFunctions::PrintTour(desiredRoute);
	}

	vector<int> padded_tour;
	vector<Node> charger_nodes;
	for(const auto &node : _nodes)
	{
		if(node.isCharger) charger_nodes.push_back(node);
	}

	int current_node_index = 0;
	padded_tour.push_back(current_node_index);

	float full_distance = 0.f;

	int customer_nodes_serviced = 0;
	while(customer_nodes_serviced < static_cast<int>(desiredRoute.size()))
	{
		const int desired_route_index = desiredRoute[customer_nodes_serviced];
		Node current_node = _nodes[current_node_index];
		Node next_desired_node = _nodes[desired_route_index];
		
		const int demand_cost = next_desired_node.demand;

		RouteType route_type;
		vector<Node> subgraph = {charger_nodes.begin(), charger_nodes.end()};
		subgraph.push_back(current_node);

		if(demand_cost <= currentInventoryCapacity)
		{
			route_type = RouteToCustomer;
			subgraph.push_back(next_desired_node);
		}
		else
		{
			route_type = RouteToDepot;
			subgraph.push_back(_nodes[0]);
		}

		PathfindingResult result;
		const vector<Node> safe_route = pathfinding(subgraph, current_node, next_desired_node, result);

		if(result == ImpossibleRoute)
		{
			cout << "Impossible route detected" << endl;
			full_distance += 1000000000.f;
			return full_distance;
		}

		for(size_t i = 1; i < safe_route.size(); i++)
		{
			padded_tour.push_back(safe_route[i].index);
			currentBatteryCapacity -= BatteryCost(safe_route[i-1], safe_route[i]);
			full_distance += HelperFunctions::CalculateInterNodeDistance(safe_route[i-1], safe_route[i]);
			if(safe_route[i].isCharger)
			{
				currentBatteryCapacity = maxBatteryCapacity;
			}
		}

		if(route_type == RouteToCustomer)
		{
			current_node_index = desired_route_index;
			currentInventoryCapacity -= demand_cost;
			customer_nodes_serviced++;
		}
		else
		{
			current_node_index = 0;
			currentInventoryCapacity = maxInventoryCapacity;
			currentBatteryCapacity = maxBatteryCapacity;
		}
	}

	vector<Node> subgraph = {charger_nodes.begin(), charger_nodes.end()};
	const Node current_node = _nodes[current_node_index];
	const Node depot = _nodes[0];
	subgraph.push_back(current_node);
	subgraph.push_back(depot);

	PathfindingResult result;
	const vector<Node> safe_route = pathfinding(subgraph, current_node, depot, result);

	if(result == ImpossibleRoute)
	{
		cout << "Impossible route detected" << endl;
		full_distance += 1000000000.f;
		return full_distance;
	}

	for(size_t i = 1; i < safe_route.size(); i++)
	{
		padded_tour.push_back(safe_route[i].index);
		currentBatteryCapacity -= BatteryCost(safe_route[i-1], safe_route[i]);
		full_distance += HelperFunctions::CalculateInterNodeDistance(safe_route[i-1], safe_route[i]);
		if(safe_route[i].isCharger)
		{
			currentBatteryCapacity = maxBatteryCapacity;
		}
	}

	float true_distance = CalculateFullRouteDistance(padded_tour);
	
	if(fabs(true_distance - full_distance) < 1)
	{
		cout << "Great, i calculated things correctly" << endl;
	}

	cout << "----------------------------------------" << endl;
	cout << "True route: ";
	for (const auto i : padded_tour)
	{
		cout << i << " ";
	}
	cout << endl;
	cout << "----------------------------------------" << endl;
	
	return true_distance;
}
*/
vector<Node> Vehicle::pathfinding(const vector<Node>& graph, const Node& start, const Node& end, PathfindingResult &out_result)
{
	float current_battery = currentBatteryCapacity;
	bool found_route_to_end = false;

	Node current_node = start;
	vector<Node> visited_nodes = {start};
	
	while(!found_route_to_end)
	{
		vector<Node> unvisited_nodes;
		//get all unvisited nodes
		for(const auto &node : graph)
		{
			bool node_has_been_visited = false;
			for(const auto &n : visited_nodes)
			{
				if(node.index == n.index)
				{
					node_has_been_visited = true;
				}
			}
			if(!node_has_been_visited)
			{
				unvisited_nodes.push_back(node);
			}
		}
		
		vector<Node> nodes_in_range = GetAllNodesWithinRange(unvisited_nodes, current_node, current_battery);

		if(nodes_in_range.empty())
		{
			out_result = ImpossibleRoute;
			return {};
		}

		//if we found a direct path from where we are to the end, then we want to go there
		for(const auto &n : nodes_in_range)
		{
			if(n.index == end.index)
			{
				//we can get to the end, but can we get there safely?
				if(CanGetToNextCustomerSafely(current_node, end))
				{
					found_route_to_end = true;
					visited_nodes.push_back(end);
					break;
				}

				//we can't get there safely, so we need to go to the nearest charger again
				found_route_to_end = false;
			}
		}
		
		//if we didn't find a direct path from where we are to the end, then we must be at a charging station
		if(!found_route_to_end)
		{
			//if we didn't find a route to the end, AND we have only not visited 1 node, AND  that node happens to be the end node,
			//we can't reach the end node, so this is an impossible path 
			if(unvisited_nodes.size() == 1 && unvisited_nodes[0].index == end.index)
			{
				cout << "We have been to every charging node and have been unable to find our way to the end" << endl;
				out_result = ImpossibleRoute;
				return {};
			}
			
			//find the closest charging node to the end that's in range 
			const Node closest = GetClosestNodeFromRange(nodes_in_range, end);

			//simulate a recharge
			current_battery = maxBatteryCapacity;

			//go to closest node
			visited_nodes.push_back(closest);
			current_node = closest;
		}
	}

	//we started at the start, and found a direct route to the end (only visited start and end nodes)
	if(visited_nodes.size() == 2)
	{
		out_result = DirectPathFound;
	}
	//we didn't find a direct path from start to end
	else
	{
		out_result = RouteThroughChargers;
	}

	return visited_nodes;
}


float Vehicle::SimulateDrive(const vector<int> &desiredRoute, bool verbose)
{
	//Resets the vehicle so that we are "starting fresh" every time a new drive starts
	ResetVehicle();
	if (verbose)
	{
		cout << "Simulating drive of route: ";
		HelperFunctions::PrintTour(desiredRoute);
	}

	//we will construct the "true" tour during the simulation. desiredRoute is just the order of customer nodes for this route.
	//we will need to find when we need to charge at a station and when we need to return to the depot as we "simulate" the drive.
	vector<int> paddedTour;

	//we start at the depot, which is always node "0"
	//starting at the depot satisfies part of the first constraint of the EVRP. We also need to end at the depot
	int currentNodeIndex = 0;
	paddedTour.push_back(currentNodeIndex);

	//start with zero distance
	float fullDistance = 0.f;

	//loop until we have serviced all customer nodes, which satisfies constraint number 2
	int customerNodesServiced = 0;
	while (static_cast<size_t>(customerNodesServiced) < desiredRoute.size())
	{
		if (verbose)
		{
			cout << "I can travel " << currentBatteryCapacity << " units and I have " << currentInventoryCapacity << " inventory remaining" << endl;
			cout << "Current Node Index: " << currentNodeIndex << endl;
		}

		
		//This if statement checks for a supposed infinite loop. It is possible, under the current implementation, to have an impossible route
		//where the vehicle must bounce between two charging stations, never having enough battery to safely go to the next customer. 
		//If that's the case, we want to heavily punish this route with a huge distance penalty. 
		if (paddedTour.size() > 10 &&
			(paddedTour[paddedTour.size() - 1] == paddedTour[paddedTour.size() - 3] && paddedTour[paddedTour.size() - 2] == paddedTour[paddedTour.size() - 4]))
		{
			//infinite loop of charging stations detected, break out and heavily penalize impossible route
			fullDistance += 1000000000.f;
			return fullDistance;
		}
		
		//We want to select the next customer node in the desired route to set as our preferred target
		int desiredRouteIndex = desiredRoute[customerNodesServiced];
		Node currentNode = _nodes[currentNodeIndex];
		Node nextDesiredNode = _nodes[desiredRouteIndex];

		//Calculate battery cost to go from the current node to the next desired node
		float routeCost = BatteryCost(currentNode, nextDesiredNode);
		const int demandCost = nextDesiredNode.demand;

		if (verbose)
		{
			cout << "Going from node (" << currentNode.x << ", " << currentNode.y << ") to node (" << nextDesiredNode.x << ", " << nextDesiredNode.y << ") will cost " << routeCost << " battery. This node has a demand of: " << demandCost << endl;
		}

		//we only want to go to the next customer node if we can get from where we are, to the next customer, then to a charging station.
		//If we can't go from the next customer to a charging station, we need to first visit a charging station or else we would get stranded
		//at the next customer. Bad day. 
		const bool canGetToNextCustomerSafely = CanGetToNextCustomerSafely(currentNode, nextDesiredNode);
		if (canGetToNextCustomerSafely)
		{
			if (verbose)
			{
				cout << "I can go to this and have the battery to continue to the nearest node (aka I won't get stranded at this node)" << endl;
			}

			//Do we have the capacity to satisfy the demand at the next customer, and can we make the trip on our current battery state
			if (currentInventoryCapacity >= demandCost && currentBatteryCapacity > routeCost)
			{
				if (verbose)
				{
					cout << "I went to this node!" << endl;
				}
				//travel to the next customer, subtracting the demand from our inventory and the battery cost from our battery capacity
				currentInventoryCapacity -= demandCost;
				currentBatteryCapacity -= routeCost;
				paddedTour.push_back(desiredRouteIndex);
				currentNodeIndex = desiredRouteIndex;
				customerNodesServiced++;
			}
			//we can't satisfy the next customer, either because we don't have enough battery to reach them or we don't have enough inventory
			else
			{
				//if we are lacking in inventory, we must visit the depot to restock. 
				if (currentInventoryCapacity < demandCost)
				{
					if (verbose)
					{
						cout << "I can't meet the demand at the next node, so I must go to the depot and restock." << endl;
					}

					//while we aren't at the depot, continue moving towards the depot. 
					while (currentNodeIndex != 0)
					{
						currentNode = _nodes[currentNodeIndex];
						nextDesiredNode = _nodes[0];

						routeCost = BatteryCost(currentNode, nextDesiredNode);
						if (currentBatteryCapacity > routeCost)
						{
							//simulate driving cost
							currentBatteryCapacity -= routeCost;

							//restock inventory and recharge. one constant in the EVRP problem is there is always a charger at the depot
							//if we introduced charging time/restock time/time windows for servicing, we wouldn't necessarily want to charge
							//every time we went to the depot, because that might be unnecessary, but in the time-doesn't-matter version of this problem
							//we might as well recharge when we restock
							currentInventoryCapacity = maxInventoryCapacity;
							currentBatteryCapacity = maxBatteryCapacity;

							paddedTour.push_back(0);
							currentNodeIndex = 0;
						}
						else
						{
							//visit the nearest charging station and continue moving towards the depot. 
							int chargingNodeIndex = GetClosestChargingStationToNode(currentNode);
							currentBatteryCapacity = maxBatteryCapacity;
							currentNodeIndex = chargingNodeIndex;
							paddedTour.push_back(chargingNodeIndex);
						}
					}

				}
				else 
				{
					//should never reach this line of code
					cout << "I am lost as a vehicle... how did I get here?" << endl;
				}
			}
		}
		//we can get to the next customer on our current battery, but once we get there we won't have enough battery to get anywhere else. 
		//we must recharge before continuing safely. 
		else
		{
			if (verbose)
			{
				cout << "I need to detour to a charging station before visiting this node." << endl;
			}
			//recharge the battery from the next charging station. This satisfies constraint 4, 5, 6, and 7. 
			int chargingNodeIndex = GetClosestChargingStationToNode(currentNode);
			currentBatteryCapacity = maxBatteryCapacity;
			currentNodeIndex = chargingNodeIndex;
			paddedTour.push_back(chargingNodeIndex);
		}
		
		if (verbose)
		{
			cout << "----------------------------------------" << endl;
		}
	}

	//we exit the while loop when we are at the last desired customer node, but we need to simulate getting home, as required by constraint 1.
	//this is going to take the same structure as the loop we used above to get to the depot. I'm sure there's some clever way to just combine them
	//but.... 
	while (currentNodeIndex != 0)
	{
		const Node currentNode = _nodes[currentNodeIndex];
		const Node desiredNode = _nodes[0];

		const float routeCost = BatteryCost(currentNode, desiredNode);
		if (currentBatteryCapacity > routeCost)
		{
			currentBatteryCapacity -= routeCost;
			paddedTour.push_back(0);
			currentNodeIndex = 0;
		}
		else
		{
			int chargingNodeIndex = GetClosestChargingStationToNode(currentNode);
			currentBatteryCapacity = maxBatteryCapacity;
			currentNodeIndex = chargingNodeIndex;
			paddedTour.push_back(chargingNodeIndex);
		}
	}

	//we have now serviced every customer node and have driven back home, and constructed the true tour
	if (verbose)
	{
		cout << "True route: ";
		for (const auto i : paddedTour)
		{
			cout << i << " ";
		}
		cout << endl;
		cout << "----------------------------------------" << endl;
	}

	//the full distance is the "fitness" of this solution
	fullDistance = CalculateFullRouteDistance(paddedTour, false);

	return fullDistance;
}



/* This new implementation of SimulateDrive is still in active development and currently won't compile.
 * There were some inefficiencies with the above implementation, so this was an attempt to clean up the fitness
 * calculation before implementing new stuff like time window constraints
float Vehicle::SimulateDrive(const vector<int> &desiredRoute, bool verbose)
{
	ResetVehicle();
	if(verbose)
	{
		cout << "Simulating drive of route: ";
		HelperFunctions::PrintTour(desiredRoute);
	}

	vector<int> padded_tour;
	vector<Node> charger_nodes;
	for(const auto &node : _nodes)
	{
		if(node.isCharger) charger_nodes.push_back(node);
	}

	int current_node_index = 0;
	padded_tour.push_back(current_node_index);

	float full_distance = 0.f;

	int customer_nodes_serviced = 0;
	while(customer_nodes_serviced < static_cast<int>(desiredRoute.size()))
	{
		int desired_route_index = desiredRoute[customer_nodes_serviced];
		Node current_node = _nodes[current_node_index];
		Node next_desired_node = _nodes[desired_route_index];

		const float route_cost = BatteryCost(current_node, next_desired_node);
		const int demand_cost = next_desired_node.demand;

		//i know that i can just go from where i am to where i want to go
		if(currentBatteryCapacity > route_cost)
		{
			
		}
		else
		{
			vector<Node> subgraph = {charger_nodes.begin(), charger_nodes.end()};
			subgraph.push_back(current_node);
			subgraph.push_back(next_desired_node);
			vector<Node> safe_route = astar_pathfinding(subgraph, current_node, next_desired_node);
		}
	}
}

struct Compare
{
	bool operator()(const pair<Node, float> &a, const pair<Node, float> &b) const
	{
		return a.second > b.second;
	}
};

vector<Node> Vehicle::astar_pathfinding(const vector<Node>& graph, const Node& start, const Node& end)
{
	priority_queue<pair<Node, float>, vector<pair<Node, float>>, Compare> pq;
	unordered_map<Node, float> cost;
	unordered_map<Node, Node> parent;

	pq.push({start, 0.f});
	cost[start] = 0.f;
	parent[start] = start;

	while(!pq.empty())
	{
		Node current = pq.top().first;
		pq.pop();

		if(current.x == end.x && current.y == end.y)
		{
			vector<Node> path;
			while(current.x != start.x || current.y != start.y)
			{
				path.push_back(current);
				current = parent[current];
			}
			path.push_back(start);
			reverse(path.begin(), path.end());
			return path;
		}

		for(const Node &neighbor : graph)
		{
			if(neighbor.x == current.x && neighbor.y == current.y) continue;

			float new_cost = cost[current] + HelperFunctions::CalculateInterNodeDistance(current, neighbor);

			if(!cost.count(neighbor) || new_cost < cost[neighbor])
			{
				cost[neighbor] = new_cost;
				pq.push({neighbor, new_cost + HelperFunctions::CalculateInterNodeDistance(neighbor, end)});
				parent[neighbor] = current;
			}
		}
	}

	return {};
}
*/

/**
* Helper for finding the closest charging station to the given node.
* 
* @param node This is the node for which we are trying to find the nearest charging station
* 
* @return The index of the nearest charging station to the given node. 
*/
int Vehicle::GetClosestChargingStationToNode(const Node &node) const
{
	float closest = numeric_limits<float>::max();
	int closestChargerIndex = -1;

	for (size_t i = 0; i < _nodes.size(); i++)
	{
		if (_nodes[i].isCharger && _nodes[i].index != node.index)
		{
			const float dist = HelperFunctions::CalculateInterNodeDistance(node, _nodes[i]);
			if (dist < closest)
			{
				closestChargerIndex = static_cast<int>(i);
				closest = dist;
			}
		}
	}
	return closestChargerIndex;
}

/**
* Checks to see if the Vehicle can safely go from one node to another with the battery cost and distance.
* 
* Safety is defined by the Vehicle going from the from node, to the to node, with enough battery left to go from the to node to a charging station.
* It doesn't necessarily mean the Vehicle MUST go to a charging station after getting to the to node, we just want to make sure that the Vehicle
* won't get stranded at the to node without enough battery to make it anywhere else.
* 
* @param from The node the Vehicle starts at
* @param to The node the Vehicle wants to go to
* 
* @return Whether or not the Vehicle can safely get from one node to the next. 
*/
bool Vehicle::CanGetToNextCustomerSafely(const Node &from, const Node &to) const
{
	const int chargerIndex = GetClosestChargingStationToNode(to);

	if (chargerIndex == -1)
	{
		return false;
	}

	const Node closestCharger = _nodes[chargerIndex];
	if (currentBatteryCapacity > BatteryCost(from, to) + BatteryCost(to, closestCharger))
	{
		return true;
	}

	return false;
}

/**
* Calculates the battery cost between two nodes, factoring in the battery consumption rate
* 
* @param node1 The first node
* @param node2 The second node
* 
* @return The battery cost between both nodes. 
*/
float Vehicle::BatteryCost(const Node &node1, const Node &node2) const
{
	return HelperFunctions::CalculateInterNodeDistance(node1, node2) * batteryConsumptionRate;
}

/** 
* Calculates the inter-node distance between each node in the true tour, once the true tour has been determined by the SimulateDrive function
* 
* @param trueRoute The true route as determined by the SimulateDrive function
* @param verbose False by default, but useful for debugging the distance calculations
* 
* @return Returns the true distance of the found route. 
*/
float Vehicle::CalculateFullRouteDistance(const vector<int> &trueRoute, bool verbose) const
{
	float dist = 0.f;

	//iterate from node 1 in the true route to the end of the route
	for (size_t i = 1; i < trueRoute.size(); i++)
	{
		if (verbose)
		{
			cout << "Calculating distance starting at node " << trueRoute[i - 1] << " and going to node " << trueRoute[i] << endl;
		}
		Node currentNode = _nodes[trueRoute[i-1]];
		Node nextNode = _nodes[trueRoute[i]];

		dist += HelperFunctions::CalculateInterNodeDistance(currentNode, nextNode);
	}
	
	return dist;
}

vector<Node> Vehicle::GetAllNodesWithinRange(const vector<Node>& graph, const Node& node, const float battery) const
{
	vector<Node> nodes_in_range;
	for(const auto &n : graph)
	{
		if(HelperFunctions::CalculateInterNodeDistance(n, node) <= battery && n.index != node.index)
		{
			nodes_in_range.push_back(n);
		}
	}
	return nodes_in_range;
}

Node Vehicle::GetClosestNodeFromRange(const vector<Node>& graph, const Node& node) const
{
	float shortest_distance = numeric_limits<float>::max();
	Node closest_node = node;

	for(const auto &n : graph)
	{
		if(n.index != node.index)
		{
			const float distance = HelperFunctions::CalculateInterNodeDistance(n, node);
			if(distance < shortest_distance)
			{
				closest_node = n;
				shortest_distance = distance;
			}
		}	
	}
	return closest_node;
}




