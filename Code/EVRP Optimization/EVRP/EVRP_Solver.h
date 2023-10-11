#pragma once

#include "GraphStructure.h"

#define STR_LEN 256 /*!< STR_LEN is the maximum number of characters a filepath could be*/

constexpr char FILENAME[STR_LEN] = ".\\EVRP\\Data_Sets\\EVRP TW\\rc103c15.txt"; /*!< The filepath to the EVRP problem definition with respect to the project root directory*/

/***************************************************************************//**
 * A class used for reading the EVRP problem definition from a file then generically solving. 
 *
 * This class loads the EVRP problem definition from a file specified by #FILENAME into a 
 * custom data structure EVRP_Data. This is necessary so that EVRP_Solver can pass the data
 * into any optimization algorithm class we want, without having to associate the data with 
 * the algorithm. 
 ******************************************************************************/

class EVRP_Solver
{
public:
	EVRP_Solver();
	vector<int> SolveEVRP();

private:
	int vehicleLoadCapacity;/*!< A temporary variable to store the inventory load capacity when we are actively parsing the data file*/
	float vehicleBatteryCapacity;/*!< A temporary variable to store the battery capacity when we are actively parsing the data file*/
	float vehicleFuelConsumptionRate;/*!< A temporary variable to store the vehicle consumption rate when we are actively parsing the data file*/

	EVRP_Data data; /*!< This is the core of what the file gets loaded into. This data structure will contain all the necessary information to represent the selected EVRP problem*/
	vector<Node> nodes; /*!< A vector of all nodes in the graph. This will contain customer nodes, charging station nodes, and the depot*/
};

