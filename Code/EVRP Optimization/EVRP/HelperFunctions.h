#pragma once
#include "ProblemDefinition.h"

class HelperFunctions
{
public:
	static int RandomNumberGenerator(const int min, const int max);
	static void ShuffleVector(vector<int>& container);
	static void PrintTour(const vector<int> &tour);
	static vector<int> GenerateRandomTour(const int customerStart, const int size);
	static float CalculateInterNodeDistance(const Node& node1, const Node& node2);
	static vector<int> GetIndexEncodedTour(const vector<Node> &tour);
	static vector<Node> GetNodeDecodedTour(const ProblemDefinition *problem, const vector<int> &tour);
};

