#pragma once
#include "ProblemDefinition.h"

class Vehicle
{
public:
	/**
	* Vehicle constructor.
	* 
	* Sets internal variables used in the calculation and simulation of driving the tour, then calls ResetVehicle to make sure everything is initialized properly
	* 
	* @param problem
	*/
	Vehicle(const ProblemDefinition *problem)
	{
		problem_definition = problem;
		_nodes = problem->GetAllNodes();
		_battery = problem->GetVehicleParameters().battery_capacity;
		_inventory = problem->GetVehicleParameters().load_capacity;
		_batteryRate = problem->GetVehicleParameters().battery_consumption_rate;
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

	float SimulateDrive(const vector<Node> &route, bool verbose = false);

private:
	enum PathfindingResult
	{
		DirectPathFound,
		RouteThroughChargers,
		ImpossibleRoute
	};
	enum RouteType
	{
		RouteToCustomer,
		RouteToDepot
	};
	int GetClosestChargingStationToNode(const Node &node) const;
	bool CanGetToNextCustomerSafely(const Node &from, const Node &to) const;
	bool CanGetToNextCustomerSafely(const Node &from, const Node &to, const float battery_level) const;
	float BatteryCost(const Node &node1, const Node &node2) const;
	float CalculateFullRouteDistance(const vector<int> &trueRoute, bool verbose=false) const;
	vector<Node> GetAllNodesWithinRange(const vector<Node> &graph, const Node &node, float battery) const;
	Node GetClosestNodeFromRange(const vector<Node> &graph, const Node &node) const;
	
	vector<Node> pathfinding(const vector<Node> &graph, const Node &start, const Node &end, PathfindingResult &out_result);

	vector<Node> _nodes; /*!< All nodes in the EVRP graph*/
	float _battery; /*!< An internal variable that holds the state of the maximum battery capacity*/
	int _inventory; /*!< An internal variable that holds the state of the maximum vehicle inventory capacity*/
	float _batteryRate; /*!< An internal variable that holds the state of the rate in which the battery discharges over distance */
	const ProblemDefinition *problem_definition;

	float currentBatteryCapacity;/*!< The current battery capacity, updated during the simulation of the driving*/
	float maxBatteryCapacity; /*!< The maximum battery capacity, shouldn't change during execution*/
	int currentInventoryCapacity; /*!< The current inventory capacity, updated during the simulation of the driving*/
	int maxInventoryCapacity; /*!< The maximum inventory capacity, shouldn't change during execution*/
	float batteryConsumptionRate; /*!< The battery consumption rate, shouldn't change during execution*/
};

