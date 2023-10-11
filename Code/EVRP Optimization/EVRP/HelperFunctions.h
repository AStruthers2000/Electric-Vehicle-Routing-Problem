#pragma once
#include "GraphStructure.h"

class HelperFunctions
{
public:
	static int RandomNumberGenerator(const int min, const int max);
	static void ShuffleVector(vector<int>& container);
	static void PrintTour(const vector<int> tour);
	static vector<int> GenerateRandomTour(const int customerStart, const int size);
};

