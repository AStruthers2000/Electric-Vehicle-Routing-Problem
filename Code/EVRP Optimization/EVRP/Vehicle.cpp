#include "Vehicle.h"

float Vehicle::SimulateDrive(const std::vector<int> desiredRoute, bool verbose)
{
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
	int currentNodeIndex = 0;
	paddedTour.push_back(currentNodeIndex);

	float fullDistance = 0.f;

	int customerNodesServiced = 0;
	while (customerNodesServiced < desiredRoute.size())
	{
		if (verbose)
		{
			std::cout << "I can travel " << currentBatteryCapacity << " units and I have " << currentInventoryCapacity << " inventory remaining" << std::endl;
			std::cout << "Current Node Index: " << currentNodeIndex << std::endl;
		}

		if (paddedTour.size() > 10 &&
			(paddedTour[paddedTour.size() - 1] == paddedTour[paddedTour.size() - 3] && paddedTour[paddedTour.size() - 2] == paddedTour[paddedTour.size() - 4]))
		{
			//infinite loop of charging stations detected, break out and heavily penalize impossible route
			fullDistance += 1000000000.f;
			return fullDistance;
		}
		
		int desiredRouteIndex = desiredRoute[customerNodesServiced];
		Node currentNode = _nodes[currentNodeIndex];
		Node nextDesiredNode = _nodes[desiredRouteIndex];

		float routeCost = BatteryCost(currentNode, nextDesiredNode);
		float demandCost = nextDesiredNode.demand;

		if (verbose)
		{
			std::cout << "Going from node (" << currentNode.x << ", " << currentNode.y << ") to node (" << nextDesiredNode.x << ", " << nextDesiredNode.y << ") will cost " << routeCost << " battery. This node has a demand of: " << demandCost << std::endl;
		}

		bool canGetToNextCustomerSafely = CanGetToNextCustomerSafely(currentNode, nextDesiredNode);
		if (canGetToNextCustomerSafely)
		{
			if (verbose)
			{
				std::cout << "I can go to this and have the battery to continue to the nearest node (aka I won't get stranded at this node)" << std::endl;
			}

			if (currentInventoryCapacity >= demandCost && currentBatteryCapacity > routeCost)
			{
				if (verbose)
				{
					std::cout << "I went to this node!" << std::endl;
				}
				currentInventoryCapacity -= demandCost;
				currentBatteryCapacity -= routeCost;
				paddedTour.push_back(desiredRouteIndex);
				currentNodeIndex = desiredRouteIndex;
				customerNodesServiced++;
			}
			else
			{
				if (currentInventoryCapacity < demandCost)
				{
					if (verbose)
					{
						std::cout << "I can't meet the demand at the next node, so I must go to the depot and restock." << std::endl;
					}
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
							int chargingNodeIndex = GetClosestChargingStationToNode(currentNode);
							currentBatteryCapacity = maxBatteryCapacity;
							currentNodeIndex = chargingNodeIndex;
							paddedTour.push_back(chargingNodeIndex);
						}
					}

				}
				else 
				{
					std::cout << "I am lost as a vehicle... how did I get here?" << std::endl;
				}
			}
		}
		else
		{
			if (verbose)
			{
				std::cout << "I need to detour to a charging station before visiting this node." << std::endl;
			}
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

	//we exit the while loop when we are at the last desired customer node, but we need to simulate getting home
	//this is going to take the same structure as the loop above. I'm sure there's some clever way to just combine them
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
	fullDistance = CalculateFullRouteDistance(paddedTour, false);

	return fullDistance;
}

float Vehicle::CalculateInterNodeDistance(const Node& node1, const Node& node2) const
{
	float dist = hypot(node1.x - node2.x, node1.y - node2.y);
	return dist;
}

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

float Vehicle::BatteryCost(const Node node1, const Node node2) const
{
	return CalculateInterNodeDistance(node1, node2) * batteryConsumptionRate;
}

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
