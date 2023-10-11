#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <numeric>
#include <limits>
#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>
#include <chrono>
#include <map>

/***************************************************************************//**
 * Data structure definitions useful in optimizing the EVRP.
 *
 * I define the two useful data structures here. I also include every library I need for this project here,
 * which I know isn't the best practice, but since every class that has anything to do with solving this problem
 * needs to include this class to get a reference to the EVRP_Data typedef, I figured that it would be a good way
 * to guarantee that I don't have multiple of the same includes spread across multiple files.
 ******************************************************************************/

using namespace std;

/** 
* The Node structure consists of an x and y coordinate, the value of the demand at this node (always 0 for charging nodes),
* a bool that represents if this node is a charger or not (depot and charging nodes both have demand = 0, so we need more specificity),
* and the index of this node in the list of all nodes. The depot is always index = 0. 
*/
typedef struct
{
	double x;
	double y; 
	int demand;
	bool isCharger;
	int index;
} Node;

/**
* the EVRP_Data structure holds a vector of every Node in the problem, as well as information for the vehicle.
* This is the core data structure that can be passed to any optimization class. All of the required information
* to start solving this problem is contained here, so this is the only payload that would need to be sent to 
* classes that implement optimization algorithms. 
*/
typedef struct
{
	vector<Node> nodes;
	float fuelCapacity;
	int loadCapacity;
	float fuelConsumptionRate;
	int customerStartIndex;
} EVRP_Data;




