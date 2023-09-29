#pragma once
#include "GraphStructure.h"
class Vehicle
{
public:
	Vehicle(const std::vector<Node> nodes, const float battery, const int inventory, const float batteryRate)
	{
		_nodes = nodes;
		_battery = battery;
		_inventory = inventory;
		_batteryRate = batteryRate;
		ResetVehicle();
	}
	~Vehicle()
	{
		desiredRoute.clear();
	}

	void ResetVehicle()
	{
		currentBatteryCapacity = _battery;
		maxBatteryCapacity = _battery;
		currentInventoryCapacity = _inventory;
		maxInventoryCapacity = _inventory;
		batteryConsumptionRate = _batteryRate;
		distanceTravelled = 0.f;
		desiredRoute.clear();
	}

	float SimulateDrive(const std::vector<int> desiredRoute, bool verbose=false);

private:
	float CalculateInterNodeDistance(const Node& node1, const Node& node2) const;
	int GetClosestChargingStationToNode(const Node node) const;
	bool CanGetToNextCustomerSafely(Node from, Node to);
	float BatteryCost(const Node node1, const Node node2) const;
	float CalculateFullRouteDistance(const std::vector<int> trueRoute, bool verbose=false);

	std::vector<Node> _nodes;
	float _battery;
	int _inventory;
	float _batteryRate;

	float currentBatteryCapacity;
	float maxBatteryCapacity;
	int currentInventoryCapacity;
	int maxInventoryCapacity;
	float batteryConsumptionRate;
	std::vector<int> desiredRoute;
	float distanceTravelled;
};

