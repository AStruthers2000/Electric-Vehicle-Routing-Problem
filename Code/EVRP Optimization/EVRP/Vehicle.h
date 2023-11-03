#pragma once
#include "GraphStructure.h"
class Vehicle
{
public:
	/**
	* Vehicle constructor.
	* 
	* Sets internal variables used in the calculation and simulation of driving the tour, then calls ResetVehicle to make sure everything is initialized properly
	* 
	* @param nodes All nodes in the EVRP graph
	* @param battery The maximum capacity of the EV battery
	* @param inventory The maximum size of the vehicle's carrying capacity
	* @param batteryRate The rate in which the battery discharges over distance
	*/
	Vehicle(const vector<Node> &nodes, const float battery, const int inventory, const float batteryRate)
	{
		_nodes = nodes;
		_battery = battery;
		_inventory = inventory;
		_batteryRate = batteryRate;
		ResetVehicle();
	}

	/**
	* Resets the vehicle to the initial state using internal private parameters. 
	*/
	void ResetVehicle()
	{
		currentBatteryCapacity = _battery;
		maxBatteryCapacity = _battery;
		currentInventoryCapacity = _inventory;
		maxInventoryCapacity = _inventory;
		batteryConsumptionRate = _batteryRate;
	}

	float SimulateDrive(const vector<int> &desiredRoute, bool verbose = false);

private:
	
	int GetClosestChargingStationToNode(const Node &node) const;
	bool CanGetToNextCustomerSafely(const Node &from, const Node &to) const;
	float BatteryCost(const Node &node1, const Node &node2) const;
	float CalculateFullRouteDistance(const vector<int> &trueRoute, bool verbose=false) const;
	vector<Node> astar_pathfinding(const vector<Node> &graph, const Node &start, const Node &end);

	vector<Node> _nodes; /*!< All nodes in the EVRP graph*/
	float _battery; /*!< An internal variable that holds the state of the maximum battery capacity*/
	int _inventory; /*!< An internal variable that holds the state of the maximum vehicle inventory capacity*/
	float _batteryRate; /*!< An internal variable that holds the state of the rate in which the battery discharges over distance */

	float currentBatteryCapacity;/*!< The current battery capacity, updated during the simulation of the driving*/
	float maxBatteryCapacity; /*!< The maximum battery capacity, shouldn't change during execution*/
	int currentInventoryCapacity; /*!< The current inventory capacity, updated during the simulation of the driving*/
	int maxInventoryCapacity; /*!< The maximum inventory capacity, shouldn't change during execution*/
	float batteryConsumptionRate; /*!< The battery consumption rate, shouldn't change during execution*/
};

