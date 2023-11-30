#include "Vehicle.h"

#include <cassert>
#include <iostream>
#include <queue>

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
* @param route The tour through just the customer nodes.
* @param verbose This is false by default, and will hide a lot of the print outputs. This should not be set to true unless you want to output one specific route. In general, the outputting adds a lot of time to the simulation execution
* 
* @return Returns the true distance that the desired route would actually traverse with the fuel and capacity constraints
*/

float Vehicle::SimulateDrive(const vector<Node> &route, bool verbose)
{
	vector<int> encoded_route = HelperFunctions::GetIndexEncodedTour(route);
	ResetVehicle();
	if(verbose)
	{
		cout << "Simulating drive of ";
		HelperFunctions::PrintTour(encoded_route);
	}

	const vector<Node> charger_nodes = problem_definition->GetChargingNodes();

	vector<int> padded_tour;
	//we start the padded tour at the depot (or node 0)
	padded_tour.push_back(0);

	//we will track the full distance of the route in case there's any early returns
	float full_distance = 0.f;
	float route_time = 0.f;
	
	int current_node_index = 0;
	int customer_nodes_serviced = 0;

	//the true desired route should be the desired route plus the depot at the very end
	vector<int> desired_route = {encoded_route.begin(), encoded_route.end()};
	desired_route.push_back(0);
	
	while(customer_nodes_serviced < static_cast<int>(desired_route.size()))
	{
		const int desired_route_index = desired_route[customer_nodes_serviced];
		Node current_node = problem_definition->GetNodeFromIndex(current_node_index);
		Node next_desired_node = problem_definition->GetNodeFromIndex(desired_route_index);
		
		const int demand_cost = next_desired_node.demand;
		const float time_cost = next_desired_node.service_time;

		const float ready_time = next_desired_node.ready_time;
		const float due_time = next_desired_node.due_date;

		if(verbose) cout << "I am currently at node " << current_node.index << " and my goal is to go to node " << next_desired_node.index << endl;
		if(verbose) cout << "The next node has a demand cost of " << demand_cost << " and I have " << currentInventoryCapacity << " inventory" << endl;
		const RouteType route_type = demand_cost <= currentInventoryCapacity ? RouteToCustomer : RouteToDepot;

		PathfindingResult result;
		vector<Node> safe_route;
		if(route_type == RouteToCustomer)
		{
			safe_route = pathfinding(charger_nodes, current_node, next_desired_node, result);
			if(verbose) cout << "I am routing to customer " << next_desired_node.index << " because I have the inventory capacity" << endl;
		}
		else
		{
			safe_route = pathfinding(charger_nodes, current_node, problem_definition->GetDepotNode(), result);
			if(verbose) cout << "I need to stop at the depot before I go to customer " << next_desired_node.index << endl;
		}

		if(result == ImpossibleRoute)
		{
			if(verbose) cout << "=!=!= Impossible route detected after regular pathfinding =!=!=" << endl;
			full_distance += 1000000000.f;
			return full_distance;
		}

		for(size_t i = 1; i < safe_route.size(); i++)
		{
			if(verbose) cout << "\tMy route has me going from node " << safe_route[i-1].index << " to node " << safe_route[i].index << endl;
			padded_tour.push_back(safe_route[i].index);
			currentBatteryCapacity -= BatteryCost(safe_route[i-1], safe_route[i]);
			route_time += TimeCost(safe_route[i-1], safe_route[i]);
			full_distance += HelperFunctions::CalculateInterNodeDistance(safe_route[i-1], safe_route[i]);
			if(safe_route[i].isCharger)
			{
				if(verbose) cout << "\t\tNode " << safe_route[i].index << " is a charging station, so I need to fuel up" << endl;
				route_time += RefuelingTime(currentBatteryCapacity);
				currentBatteryCapacity = maxBatteryCapacity;
			}
		}

		if(route_type == RouteToCustomer)
		{
			//outside time window, bad
			if(route_time < ready_time || route_time > due_time)
			{
				//add some route punishment
				//full_distance += 10000;
			}

			route_time += time_cost;
			current_node_index = desired_route_index;
			currentInventoryCapacity -= demand_cost;
			
			customer_nodes_serviced++;
			if(verbose) cout << "I am now at node " << current_node_index << " and have serviced this customer" << endl;
			assert(currentInventoryCapacity >= 0);
		}
		else if(route_type == RouteToDepot)
		{
			current_node_index = 0;
			//reset the route time, aka new vehicle leaving the depot at t = 0
			route_time = 0;
			currentInventoryCapacity = maxInventoryCapacity;
			route_time += RefuelingTime(currentBatteryCapacity);
			currentBatteryCapacity = maxBatteryCapacity;
			if(verbose) cout << "I made it to the depot, and have refilled my inventory and my battery capacity" << endl;
		}

		assert(currentBatteryCapacity >= 0);
		assert(currentInventoryCapacity >= 0);
		if(verbose) cout << "-------------------------------------------------------" << endl;
	}

	//const float true_distance = CalculateFullRouteDistance(padded_tour);
	//cout << "True distance: " << true_distance << ", and \"simulated\" full distance: " << full_distance << endl;
	//assert(fabs(true_distance - full_distance) < 1);

	if(verbose)
	{
		cout << "----------------------------------------" << endl;
		cout << "True route with distance " << full_distance << ": ";
		for (const auto i : padded_tour)
		{
			cout << i << " ";
		}
		cout << endl;
		cout << "----------------------------------------" << endl;
	}
	return full_distance;
}

/**
 * \brief 
 * \param graph 
 * \param start 
 * \param end 
 * \param out_result 
 * \return 
 */
vector<Node> Vehicle::pathfinding(const vector<Node>& graph, const Node& start, const Node& end, PathfindingResult &out_result)
{
	float current_battery = currentBatteryCapacity;
	bool found_route_to_end = false;

	Node current_node = start;
	vector<Node> visited_nodes = {start};
	//cout << "== Pathfinding ==" << endl;
	while(!found_route_to_end)
	{
		//cout << "My current range is " << current_battery << endl;
		//cout << "Distance to the next node is " << BatteryCost(current_node, end) << endl;
		//if we can go from the current node to the end safely, we are done
		if(CanGetToNextCustomerSafely(current_node, end, current_battery))
		{
			//cout << "Can get from node " << current_node.index << " to node " << end.index << " safely without charging" << endl;
			//cout << "I have capacity " << current_battery << " and the distance cost is " << BatteryCost(current_node, end) << endl;
			found_route_to_end = true;
			visited_nodes.push_back(end);
		}
		else
		{
			//cout << "I can't get from node " << current_node.index << " to node " << end.index << " safely without charging" << endl;
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
			//cout << "There are " << nodes_in_range.size() << " nodes within my current battery range" << endl;
			if(nodes_in_range.empty())
			{
				cout << "Oh... I am stranded. Oops ig" << endl;
				out_result = ImpossibleRoute;
				return {};
			}

			//find the closest charging node to the end that's in range 
			const Node closest = GetClosestNodeFromRange(nodes_in_range, end);
			//cout << "The closest node to the end that I can get to is node " << closest.index << endl;

			//simulate a recharge
			current_battery = maxBatteryCapacity;

			//go to closest node
			visited_nodes.push_back(closest);
			current_node = closest;
		}
		
		//cout << "=======================================" << endl;
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

	//cout << "== Pathfinding ==" << endl;

	return visited_nodes;
}

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

	const vector<Node> charging_nodes = problem_definition->GetChargingNodes();
	for (const auto& charging_node : charging_nodes)
	{
		if (charging_node.index != node.index)
		{
			const float dist = HelperFunctions::CalculateInterNodeDistance(node, charging_node);
			if (dist < closest)
			{
				closestChargerIndex = charging_node.index;
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
	return CanGetToNextCustomerSafely(from, to, currentBatteryCapacity);
}

bool Vehicle::CanGetToNextCustomerSafely(const Node& from, const Node& to, const float battery_level) const
{
	const int chargerIndex = GetClosestChargingStationToNode(to);
	//cout << "\tThe closest charger to node " << to.index << " is node " << chargerIndex << endl;

	if (chargerIndex == -1)
	{
		return false;
	}

	const Node closestCharger = problem_definition->GetNodeFromIndex(chargerIndex);
	if (battery_level > BatteryCost(from, to) + BatteryCost(to, closestCharger))
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
 * \brief 
 * \param node1 
 * \param node2 
 * \return 
 */
float Vehicle::TimeCost(const Node& node1, const Node& node2) const
{
	return HelperFunctions::CalculateInterNodeDistance(node1, node2) * _averageVelocity;
}

/**
 * \brief 
 * \param battery_level 
 * \return 
 */
float Vehicle::RefuelingTime(const float battery_level)
{
	const float difference = maxBatteryCapacity - battery_level;
	return difference / _inverseRefuelingRate;
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
		Node currentNode = problem_definition->GetNodeFromIndex(trueRoute[i-1]);
		Node nextNode = problem_definition->GetNodeFromIndex(trueRoute[i]);

		dist += HelperFunctions::CalculateInterNodeDistance(currentNode, nextNode);
	}
	
	return dist;
}

/**
 * \brief 
 * \param graph 
 * \param node 
 * \param battery 
 * \return 
 */
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

/**
 * \brief 
 * \param graph 
 * \param node 
 * \return 
 */
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