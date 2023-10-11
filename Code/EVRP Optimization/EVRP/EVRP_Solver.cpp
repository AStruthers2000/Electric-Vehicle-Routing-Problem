#include "EVRP_Solver.h"
#include "Algorithms\GA\GeneticAlgorithmOptimizer.h"
#include "Algorithms/RandomSearch/RandomSearchOptimizer.h"

/***************************************************************************//**
 * EVRP_Solver constructor handles the loading of data from a file.
 *
 * The filepath of the problem is defined in the filename variable. 
 * The data consists of rows representing a node in the graph, where each node
 * is either the depot (type = d), a charging station (type = f), or a customer 
 * node (type = c). Each node has an x and y coordinate that specify its location in 
 * the graph, as well as a demand value for customer nodes. The depot and all charging 
 * stations always have demand = 0, and there is always a charging station at the depot.
 * 
 * The datasets also include information on the vehicle fuel tank capacity, which in 
 * the case of the EVRP, this is the battery capacity. There is also information 
 * denoting the maximum inventory capacity each vehicle can hold, the fuel consumption
 * rate, the inverse refueling rate (unused) and the average velocity (unused). 
 * 
 * We populate a data structure of type EVRP_Data with a vector of all nodes, the 
 * battery capacity, the inventory capacity, the fuel consumption rate, and the index
 * representing the first customer node in the vector of all nodes. 
 * 
 * After the data is loaded into the custom EVRP_Data struct, we do a quick summation
 * to find the total demand from all customer nodes, divided by the vehicle inventory capacity,
 * to determine the true miniumum number of subtours possible if the only constraint is inventory.
 ******************************************************************************/
EVRP_Solver::EVRP_Solver()
{
	ifstream file;

	file.open(FILENAME);
	if (!file.is_open())
	{
		cout << "Failed to open data file, exiting" << endl;
		exit(1);
	}
	else
	{

		string ID;
		char nodeType;
		string line;
		double x, y;
		int demand;
		int index = 0;
		data.customerStartIndex = -1;
		while (getline(file, line))
		{
			istringstream iss(line);
			if (!(iss >> ID >> nodeType >> x >> y >> demand))
			{
				char type = line[0];
				size_t pos = 0;
				string token;
				while ((pos = line.find('/')) != string::npos)
				{
					token = line.substr(0, pos);
					line.erase(0, pos + 1);
				}
				if (!token.empty())
				{
					float num = stof(token);
					switch (type)
					{
					case 'Q':
						vehicleBatteryCapacity = num;
						break;
					case 'C':
						vehicleLoadCapacity = static_cast<int>(num);
						break;
					case 'r':
						vehicleFuelConsumptionRate = num;
					default:
						break;
					}
				}
			}
			else
			{
				Node n = Node{ x, y, 0, false, index };
				switch (nodeType)
				{
				case 'f':
					n.demand = 0;
					n.isCharger = true;
					break;
				case 'c':
					if (data.customerStartIndex == -1) data.customerStartIndex = index;
					n.demand = demand;
					n.isCharger = false;
					break;
				default:
					break;
				}
				nodes.push_back(n);
				index++;
			}
		}
		data = EVRP_Data{ nodes, vehicleBatteryCapacity, vehicleLoadCapacity, vehicleFuelConsumptionRate, data.customerStartIndex };
	}
	file.close();

	int tot_demand = 0;
	for (const Node& node : nodes)
	{
		tot_demand += node.demand;
	}
	cout << "The minimum number of subtours with only constraint of capacity is: " << ceil(double(tot_demand) / vehicleLoadCapacity) << endl;
}

/***************************************************************************//**
 * SolveEVRP is where the choice of algorithm occurs. 
 *
 * In order to keep the problem and the algorithm implementation seperate, the 
 * SolveEVRP function has control over which algorithm it selects. Currently, there
 * is only the GeneticAlgorithmOptimizer class in this project, but future work could
 * extend the amount of algorithms being used. To implement a different optimization
 * algorithm, all that would need to happen is to create a new class, for example 
 * AntColonyOptimizer, that has a function we can pass EVRP_Data, a vector out parameter
 * that represents the optimal tour, and a float out parameter that stores the distance of 
 * the optimal tour.
 ******************************************************************************/
vector<int> EVRP_Solver::SolveEVRP()
{
	//The only currently implemented optimization algorithm 
	GeneticAlgorithmOptimizer* ga = new GeneticAlgorithmOptimizer();

	RandomSearchOptimizer* randSearch = new RandomSearchOptimizer();

	//Out parameter for the optimal tour
	vector<int> optimalTour;

	//Out parameter for the distance of the optimal tour
	float bestDistance;

	//Function call to the GeneticAlgorithmOptimizer class that will return the best tour
	//from the given data
	//ga->Optimize(data, optimalTour, bestDistance);
	randSearch->Optimize(data, optimalTour, bestDistance);

	cout << "The best route has a distance of: " << bestDistance << endl;
	return optimalTour;
	
}
