#pragma once
#include <string>
#include <vector>



/***************************************************************************//**
 * Data structure definitions useful in optimizing the EVRP.
 *
 * I define the two useful data structures here. I also include every library I need for this project here,
 * which I know isn't the best practice, but since every class that has anything to do with solving this problem
 * needs to include this class to get a reference to the EVRP_Data typedef, I figured that it would be a good way
 * to guarantee that I don't have multiple of the same includes spread across multiple files.
 ******************************************************************************/

using namespace std;

enum NodeType
{
	Depot,
	Charger,
	Customer
};

/** 
* The Node structure consists of an x and y coordinate, the value of the demand at this node (always 0 for charging nodes),
* a bool that represents if this node is a charger or not (depot and charging nodes both have demand = 0, so we need more specificity),
* and the index of this node in the list of all nodes. The depot is always index = 0. 
*/
struct Node
{
	double x;
	double y; 
	int demand;
	NodeType node_type;
	float ready_time;
	float due_data;
	float service_time;
	int index;
	bool isCharger = false;

	bool operator==(const Node &) const;
	bool operator!=(const Node &) const;
	bool operator<(const Node &) const;
};

inline bool Node::operator==(const Node& n) const
{
	return index == n.index;
}

inline bool Node::operator!=(const Node& n) const
{
	return index != n.index;
}
inline bool Node::operator<(const Node& n) const
{
	return demand < n.demand;
}




/**
* the EVRP_Data structure holds a vector of every Node in the problem, as well as information for the vehicle.
* This is the core data structure that can be passed to any optimization class. All of the required information
* to start solving this problem is contained here, so this is the only payload that would need to be sent to 
* classes that implement optimization algorithms. 
*/
/*
typedef struct
{
	vector<Node> nodes;
	float fuelCapacity;
	int loadCapacity;
	float fuelConsumptionRate;
	int customerStartIndex;
} EVRP_Data;
*/

struct optimization_result
{
	string algorithm_name;
	float execution_time;
	float distance;
	vector<int> solution_encoded;
	vector<Node> solution_decoded;
	vector<string> hyperparameters;
	//maybe care about memory use?
};

struct VehicleParameters
{
	//inventory
	int load_capacity;

	//battery
	float battery_capacity;
	float battery_consumption_rate;
	float inverse_recharging_rate;

	//movement
	float average_velocity;
};

class ProblemDefinition
{
public:
	ProblemDefinition(const vector<Node> &nodes, const VehicleParameters &vehicle_params)
	{
		for(const auto &n : nodes)
		{
			all_nodes.push_back(n);
			switch (n.node_type) {
			case Depot:
				depot = n;
				break;
			case Charger:
				charger_nodes.push_back(n);
				break;
			case Customer:
				customer_nodes.push_back(n);
				break;
			}
		}

		vehicle_parameters = vehicle_params;
	}

	vector<Node> GenerateRandomTour() const;
	
	Node GetDepotNode() const { return depot; }
	vector<Node> GetAllNodes() const { return all_nodes; }
	vector<Node> GetChargingNodes() const { return charger_nodes; }
	vector<Node> GetCustomerNodes() const { return customer_nodes; }
	VehicleParameters GetVehicleParameters() const { return vehicle_parameters; }
	Node GetNodeFromIndex(const int index) const
	{
		for(const auto &n : GetAllNodes())
		{
			if(n.index == index)
			{
				return n;
			}
		}
		return {};
	}
	

private:
	Node depot;
	vector<Node> all_nodes;
	vector<Node> customer_nodes;
	vector<Node> charger_nodes;

	VehicleParameters vehicle_parameters;
};
