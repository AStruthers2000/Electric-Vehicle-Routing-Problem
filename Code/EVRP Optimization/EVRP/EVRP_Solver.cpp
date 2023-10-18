#include "EVRP_Solver.h"
#include "Algorithms/AlgorithmBase.h"
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

	file.open(string(DATA_PATH) + string(READ_FILENAME));
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
void EVRP_Solver::SolveEVRP()
{
	vector<AlgorithmBase*> algorithms;

	algorithms.push_back(new GeneticAlgorithmOptimizer(data));
	algorithms.push_back(new RandomSearchOptimizer(data));

	/*
	//Out parameter for the optimal tour
	vector<int> optimalTour;

	//Out parameter for the distance of the optimal tour
	float bestDistance;

	//Function call to the GeneticAlgorithmOptimizer class that will return the best tour
	//from the given data
	//ga->Optimize(data, optimalTour, bestDistance);
	rand_search->Optimize(optimalTour, bestDistance);
	

	cout << "The best route has a distance of: " << bestDistance << endl;
	return optimalTour;
	*/
	
	for(const auto alg : algorithms)
	{
		vector<int> encoded_tour;
		float best_distance;

		//What time is it before solving the problem
		const auto start_time = std::chrono::high_resolution_clock::now();
		
		alg->Optimize(encoded_tour, best_distance);

		//What time is it now that we've solved the problem
		const auto end_time = chrono::high_resolution_clock::now();

		//Get the execution time in milliseconds 
		const auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
		
		cout << "Execution time: " << static_cast<float>(duration)/1000.0f << " seconds" << endl;
		cout << "The best route has a distance of: " << best_distance << endl;

		optimization_result result;
		result.algorithm_name = alg->GetName();
		result.execution_time = static_cast<float>(duration)/1000.0f;
		result.solution_encoded = encoded_tour;
		result.distance = best_distance;
		result.hyperparameters = alg->GetHyperParameters();
		
		WriteToFile(result);
	}
}

void EVRP_Solver::WriteToFile(const optimization_result& result)
{
	ofstream file;
	file.open(WRITE_FILENAME, ios_base::app);
	file << result.distance << ",";
	file << READ_FILENAME << ",";
	file << result.algorithm_name << ",";
	file << result.execution_time << ",";

	string encoded_solution;
	for(const auto &iter : result.solution_encoded)
	{
		encoded_solution += to_string(iter) + " ";
	}
	encoded_solution.pop_back();
	file << encoded_solution << ",";

	string hyper_parameters;
	for(const auto &iter : result.hyperparameters)
	{
		hyper_parameters += iter + "|";
	}
	hyper_parameters.pop_back();
	file << hyper_parameters;
	
	file << "\n";

	file.clear();
}
