#include "EVRP_Solver.h"

#include <cassert>
#include <fstream>
#include <mutex>
#include <iostream>
#include <sstream>

#include "ProblemDefinition.h"
#include "HelperFunctions.h"
#include "SolutionSet.h"
#include "Algorithms/GA/GeneticAlgorithmOptimizer.h"
#include "Algorithms/NEH/NEH_NearestNeighbor.h"
#include "Algorithms/RandomSearch/RandomSearchOptimizer.h"


mutex file_write_mutex_;

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
 * to determine the true minimum number of subtours possible if the only constraint is inventory.
 ******************************************************************************/
EVRP_Solver::EVRP_Solver(const string &file_name)
{
	ifstream file;

	file.open(string(DATA_PATH) + file_name);
	if (!file.is_open())
	{
		cout << "Failed to open data file " << file_name << ", exiting" << endl;
		_is_good_open = false;
		return;
	}
	_current_filename = file_name;

	/*
	vector<Node> nodes;
	VehicleParameters params;
	int node_index = 0;

	string line;
	while(getline(file, line))
	{
		istringstream iss(line);
		Node node;
		string StringID;
		char type;
		if(!(iss >> StringID >> type >> node.x >> node.y >> node.demand >> node.ready_time >> node.due_data >> node.service_time))
		{
			cerr<<"Error reading nodes from file" << endl;
		}
		switch(type)
		{
		case 'f':
			node.node_type = Charger;
			node.isCharger = true;
			break;
		case 'c':
			node.node_type = Customer;
			break;
		case 'd':
			node.node_type = Depot;
			break;
		default: break;
		}
		node.index = node_index;
		node_index++;
		nodes.push_back(node);
	}

	getline(file, line);
	istringstream params_stream(line);
	if(!(params_stream >> params.battery_capacity >> params.load_capacity >> params.battery_consumption_rate >> params.inverse_recharging_rate >> params.average_velocity))
	{
		cerr << "Error reading vehicle parameters from file" << endl;
	}
	*/
	
	string ID;
	char nodeType;
	string line;
	double x, y;
	int demand;
	int index = 0;
	vector<Node> nodes;
	VehicleParameters params;
	int customerStartIndex = -1;
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
					//vehicleBatteryCapacity = num;
					params.battery_capacity = num;
					break;
				case 'C':
					//vehicleLoadCapacity = static_cast<int>(num);
					params.load_capacity = static_cast<int>(num);
					break;
				case 'r':
					//vehicleFuelConsumptionRate = num;
					params.battery_consumption_rate = num;
					break;
				default:
					break;
				}
			}
		}
		else
		{
			Node n;
			n.x = x; n.y = y;
			n.demand = demand;
			switch(nodeType)
			{
			case 'f':
				n.node_type = Charger;
				n.isCharger = true;
				break;
			case 'c':
				n.node_type = Customer;
				break;
			case 'd':
				n.node_type = Depot;
				break;
			default: break;
			}
			n.index = index;
			nodes.push_back(n);
			index++;
		}
	}
	//data = EVRP_Data{ nodes, vehicleBatteryCapacity, vehicleLoadCapacity, vehicleFuelConsumptionRate, data.customerStartIndex };
	
	file.close();

	problem_definition = new ProblemDefinition(nodes, params);
	_is_good_open = true;
	cout << "~=~=~=~= Solving problem " << file_name << " now ~=~=~=~=" << endl;
}

void EVRP_Solver::DebugEVRP() const
{
	
	auto *alg = new GeneticAlgorithmOptimizer(problem_definition);
	solution s = {};
	alg->Optimize(s);

	HelperFunctions::PrintTour(HelperFunctions::GetIndexEncodedTour(s.tour));
	cout << "Best tour has a distance of: " << s.distance << endl;
	
	/*
	auto *vehicle = new Vehicle(problem_definition);
	const vector<int> test_route = {7, 8, 4, 5, 6};
	const float result = vehicle->SimulateDrive(HelperFunctions::GetNodeDecodedTour(problem_definition, test_route), true);
	cout << "Best tour has a distance of: " << result << endl;
	*/
}

/***************************************************************************//**
 * \brief SolveEVRP is where the choice of algorithm occurs. 
 *
 * In order to keep the problem and the algorithm implementation separate, the 
 * SolveEVRP function has control over which algorithm it selects. Currently, we
 * implement GeneticAlgorithmOptimizer, RandomSearchOptimizer, and NEH_NearestNeighbor.
 * Each one of these algorithms runs with the provided problem instance, and the results
 * are each logged to a file with the proper information. 
 ******************************************************************************/
void EVRP_Solver::SolveEVRP() const
{
	vector<AlgorithmBase*> algorithms;

	//Create new instances of the algorithm solvers
	//algorithms.push_back(new GeneticAlgorithmOptimizer(data));
	//algorithms.push_back(new RandomSearchOptimizer(data));
	algorithms.push_back(new NEH_NearestNeighbor(problem_definition));
	//algorithms.push_back(algorithm(data));
	
	for(const auto alg : algorithms)
	{
		cout << "Calculating standard solve for " << alg->GetName() << "!" << endl;
		
		solution best_solution = {};
		
		//What time is it before solving the problem
		//const auto start_time = std::chrono::high_resolution_clock::now();
		HANDLE thread = GetCurrentThread();
		ULARGE_INTEGER start = get_thread_CPU_time(thread);

		//Function call to the GeneticAlgorithmOptimizer class that will return the best tour
		//from the given data
		alg->Optimize(best_solution);

		//What time is it now that we've solved the problem
		ULARGE_INTEGER end = get_thread_CPU_time(thread);
		//const auto end_time = chrono::high_resolution_clock::now();
		
		//Get the execution time in milliseconds 
		const double duration = static_cast<double>(end.QuadPart - start.QuadPart) / 10000;

		//const auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
		//const auto duration = static_cast<double>(end.QuadPart - start.QuadPart) * 1e-7;
		//const auto duration = 0.f;
		
		//cout << "Execution time of algorithm " << alg->GetName() << ": " << static_cast<float>(duration)/1000.0f << " seconds" << endl;
		//cout << "The best route has a distance of: " << best_distance << endl;

		for (const auto& index : best_solution.tour)
		{
			int index_count = 0;
			for (const auto& i : best_solution.tour)
			{
				if (index == i) index_count++;
			}
			assert(index_count == 1);
		}

		optimization_result result;
		result.algorithm_name = alg->GetName();
		result.execution_time = static_cast<float>(duration) / 1000.0f;
		result.solution_encoded = HelperFunctions::GetIndexEncodedTour(best_solution.tour);
		result.distance = best_solution.distance;
		result.hyperparameters = alg->GetHyperParameters();

		unique_lock<mutex> lock(file_write_mutex_);
		WriteToFile(result);
		lock.unlock();
	}
}

void EVRP_Solver::SolveEVRP_Seed(SeedAlgorithm seed) const
{
	vector<vector<int>> seed_solutions;
	AlgorithmBase* seed_solver = nullptr;
	switch(seed)
	{
	case NEH:
		seed_solver = new NEH_NearestNeighbor(problem_definition);
		break;
	case RNG:
		seed_solver = new RandomSearchOptimizer(problem_definition);
		break;
	}
	if(seed_solver == nullptr) return;

	cout << "Seed Solver with seed algorithm " << seed_solver->GetName() << endl;
	
	solution s = {};
	seed_solver->Optimize(s);
	cout << "Best solution has distance of: " << s.distance <<endl;

	s = {};
	
	const auto GA_solver = new GeneticAlgorithmOptimizer(problem_definition);
	GA_solver->SetSeedSolutions(seed_solver->GetFoundTours());
	GA_solver->Optimize(s);
}

/**
 * \brief Static write to file function that takes the results and writes to @WRITE_FILENAME.
 * We care about logging the results distance, name of the problem, algorithm name, and execution time.
 * We also care about writing the solution and the hyperparameters to the file in case we want to do more
 * evaluation on the specifics that went into generating this solution.
 * \param result A const reference to the optimization_results data structure that holds the information we care about
 */
void EVRP_Solver::WriteToFile(const optimization_result& result) const
{
	ofstream file;
	file.open(WRITE_FILENAME, ios_base::app);

	//The distance of the solution
	file << result.distance << ",";

	//The problem name
	file << _current_filename << ",";
	
	file << result.algorithm_name << ",";
	file << result.execution_time << ",";

	//Writing each element of the solution vector
	string encoded_solution;
	for(const auto &iter : result.solution_encoded)
	{
		encoded_solution += to_string(iter) + " ";
	}
	encoded_solution.pop_back();
	file << encoded_solution << ",";

	//Writing each element of the hyperparameters vector
	string hyper_parameters;
	for(const auto &iter : result.hyperparameters)
	{
		hyper_parameters += iter + "|";
	}
	hyper_parameters.pop_back();
	file << hyper_parameters;
	
	file << "\n";

	file.close();
}

ULARGE_INTEGER EVRP_Solver::get_thread_CPU_time(HANDLE h_thread) const
{
	FILETIME ftCreation, ftExit, ftKernel, ftUser;
	ULARGE_INTEGER ulKernel, ulUser;
	if (GetThreadTimes(h_thread, &ftCreation, &ftExit, &ftKernel, &ftUser)) {
		ulKernel.HighPart = ftKernel.dwHighDateTime;
		ulKernel.LowPart = ftKernel.dwLowDateTime;
		ulUser.HighPart = ftUser.dwHighDateTime;
		ulUser.LowPart = ftUser.dwLowDateTime;
	}
	ULARGE_INTEGER total;
	total.QuadPart = ulKernel.QuadPart + ulUser.QuadPart;
	return total;
}
