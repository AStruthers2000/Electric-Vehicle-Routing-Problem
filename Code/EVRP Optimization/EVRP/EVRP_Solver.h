#pragma once

#include "GraphStructure.h"
//#include <mutex>

enum
{
	STR_LEN = 256 /*!< STR_LEN is the maximum number of characters a filepath could be */
};

constexpr char DATA_PATH[STR_LEN] = R"(.\EVRP\Data_Sets\EVRP TW\)";
constexpr char READ_FILENAME[STR_LEN] = "c101_21.txt"; /*!< The filepath to the EVRP problem definition with respect to the project root directory */
constexpr char WRITE_FILENAME[STR_LEN] = R"(.\EVRP\Output\RawOutput.txt)";

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
	EVRP_Solver(const string &file_name);
	void SolveEVRP() const;
	bool IsGoodOpen() const { return _is_good_open;}

private:
	void WriteToFile(const optimization_result &result) const;
	
	int vehicleLoadCapacity;/*!< A temporary variable to store the inventory load capacity when we are actively parsing the data file*/
	float vehicleBatteryCapacity;/*!< A temporary variable to store the battery capacity when we are actively parsing the data file*/
	float vehicleFuelConsumptionRate;/*!< A temporary variable to store the vehicle consumption rate when we are actively parsing the data file*/

	EVRP_Data data; /*!< This is the core of what the file gets loaded into. This data structure will contain all the necessary information to represent the selected EVRP problem*/
	vector<Node> nodes; /*!< A vector of all nodes in the graph. This will contain customer nodes, charging station nodes, and the depot*/

	string _current_filename;
	bool _is_good_open;

	
};

