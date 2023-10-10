#include "Vehicle.h"

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
float Vehicle::SimulateDrive(const std::vector<int> desiredRoute, bool verbose)
{
	//Resets the vehicle so that we are "starting fresh" every time a new drive starts
	ResetVehicle();
	if (verbose)
	{
		std::cout << "Simulating drive of route: ";
		for (auto i : desiredRoute)
		{
			std::cout << i << " ";
		}
		std::cout << std::endl;
		std::cout << "----------------------------------------" << std::endl;
	}

	//we will construct the "true" tour during the simulation. desiredRoute is just the order of customer nodes for this route.
	//we will need to find when we need to charge at a station and when we need to return to the depot as we "simulate" the drive.
	std::vector<int> paddedTour;

	//we start at the depot, which is always node "0"
	//starting at the depot satisfies part of the first constraint of the EVRP. We also need to end at the depot
	int currentNodeIndex = 0;
	paddedTour.push_back(currentNodeIndex);

	//start wuth zero distance
	float fullDistance = 0.f;

	//loop until we have serviced all customer nodes, which satisfies constraint number 2
	int customerNodesServiced = 0;
	while (customerNodesServiced < desiredRoute.size())
	{
		if (verbose)
		{
			std::cout << "I can travel " << currentBatteryCapacity << " units and I have " << currentInventoryCapacity << " inventory remaining" << std::endl;
			std::cout << "Current Node Index: " << currentNodeIndex << std::endl;
		}

		/*
		This if statement checks for a supposed infinite loop. It is possible, under the current implementation, to have an impossible route
		where the vehicle must bounce between two charging stations, never having enough battery to safely go to the next customer. 
		If that's the case, we want to heavily punish this route with a huge distance penalty. 
		*/
		
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
		float demandCost = nextDesiredNode.demand;

		if (verbose)
		{
			std::cout << "Going from node (" << currentNode.x << ", " << currentNode.y << ") to node (" << nextDesiredNode.x << ", " << nextDesiredNode.y << ") will cost " << routeCost << " battery. This node has a demand of: " << demandCost << std::endl;
		}

		//we only want to go to the next customer node if we can get from where we are, to the next customer, then to a charging station.
		//If we can't go from the next customer to a charging station, we need to first visit a charging station or else we would get stranded
		//at the next customer. Bad day. 
		bool canGetToNextCustomerSafely = CanGetToNextCustomerSafely(currentNode, nextDesiredNode);
		if (canGetToNextCustomerSafely)
		{
			if (verbose)
			{
				std::cout << "I can go to this and have the battery to continue to the nearest node (aka I won't get stranded at this node)" << std::endl;
			}

			//Do we have the capacity to satisfy the demand at the next customer, and can we make the trip on our current battery state
			if (currentInventoryCapacity >= demandCost && currentBatteryCapacity > routeCost)
			{
				if (verbose)
				{
					std::cout << "I went to this node!" << std::endl;
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
						std::cout << "I can't meet the demand at the next node, so I must go to the depot and restock." << std::endl;
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
					std::cout << "I am lost as a vehicle... how did I get here?" << std::endl;
				}
			}
		}
		//we can get to the next customer on our current battery, but once we get there we won't have enough battery to get anywhere else. 
		//we must recharge before continuing safely. 
		else
		{
			if (verbose)
			{
				std::cout << "I need to detour to a charging station before visiting this node." << std::endl;
			}
			//recharge the battery from the next charging station. This satisfies constraint 4, 5, 6, and 7. 
			int chargingNodeIndex = GetClosestChargingStationToNode(currentNode);
			currentBatteryCapacity = maxBatteryCapacity;
			currentNodeIndex = chargingNodeIndex;
			paddedTour.push_back(chargingNodeIndex);
		}
		
		if (verbose)
		{
			std::cout << "----------------------------------------" << std::endl;
		}
	}

	//we exit the while loop when we are at the last desired customer node, but we need to simulate getting home, as required by constraint 1.
	//this is going to take the same structure as the loop we used above to get to the depot. I'm sure there's some clever way to just combine them
	//but.... 
	while (currentNodeIndex != 0)
	{
		Node currentNode = _nodes[currentNodeIndex];
		Node desiredNode = _nodes[0];

		float routeCost = BatteryCost(currentNode, desiredNode);
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
		std::cout << "True route: ";
		for (auto i : paddedTour)
		{
			std::cout << i << " ";
		}
		std::cout << std::endl;
		std::cout << "----------------------------------------" << std::endl;
	}

	//the full distance is the "fitness" of this solution
	fullDistance = CalculateFullRouteDistance(paddedTour, false);

	return fullDistance;
}

/**
* Calculates the distance between two nodes. Each Node has an x and y coordinate, so we can find the straight line between both points. 
* 
* @param node1 The first node in the distance calculation
* @param node2 The second node in the distance calculation
* 
* @return Returns the Euclidean distance calculation between two nodes.
*/
float Vehicle::CalculateInterNodeDistance(const Node& node1, const Node& node2) const
{
	float dist = hypot(node1.x - node2.x, node1.y - node2.y);
	return dist;
}

/**
* Helper for finding the closest charging station to the given node.
* 
* @param node This is the node for which we are trying to find the nearest charging station
* 
* @return The index of the nearest charging station to the given node. 
*/
int Vehicle::GetClosestChargingStationToNode(const Node node) const
{
	float closest = std::numeric_limits<float>::max();
	int closestChargerIndex = -1;

	for (int i = 0; i < _nodes.size(); i++)
	{
		if (_nodes[i].isCharger && _nodes[i].index != node.index)
		{
			float dist = CalculateInterNodeDistance(node, _nodes[i]);
			if (dist < closest)
			{
				closestChargerIndex = i;
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
bool Vehicle::CanGetToNextCustomerSafely(Node from, Node to)
{
	int chargerIndex = GetClosestChargingStationToNode(to);

	if (chargerIndex == -1)
	{
		return false;
	}

	Node closestCharger = _nodes[chargerIndex];
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
float Vehicle::BatteryCost(const Node node1, const Node node2) const
{
	return CalculateInterNodeDistance(node1, node2) * batteryConsumptionRate;
}

/** 
* Calculates the inter-node distance between each node in the true tour, once the true tour has been determined by the SimulateDrive function
* 
* @param trueRoute The true route as determined by the SimulateDrive function
* @param verbose False by default, but useful for debugging the distance calculations
* 
* @return Returns the true distance of the found route. 
*/
float Vehicle::CalculateFullRouteDistance(const std::vector<int> trueRoute, bool verbose)
{
	float dist = 0.f;

	//iterate from node 1 in the true route to the end of the route
	for (int i = 1; i < trueRoute.size(); i++)
	{
		if (verbose)
		{
			std::cout << "Calculating distance starting at node " << trueRoute[i - 1] << " and going to node " << trueRoute[i] << std::endl;
		}
		Node currentNode = _nodes[trueRoute[i-1]];
		Node nextNode = _nodes[trueRoute[i]];

		dist += CalculateInterNodeDistance(currentNode, nextNode);
	}
	
	return dist;
}
