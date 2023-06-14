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

typedef struct
{
	double x;
	double y;
	int demand;
	bool isCharger;
	int index;
} Node;

typedef struct
{
	std::vector<Node> nodes;
	float fuelCapacity;
	int loadCapacity;
	float fuelConsumptionRate;
	int customerStartIndex;
} EVRP_Data;