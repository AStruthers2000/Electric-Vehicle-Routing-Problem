#include "EVRP_Solver.h"
#include "GA\GAOptimizer.h"


EVRP_Solver::EVRP_Solver()
{
#if DEBUG
	capacity = 10;
	//nodes.push_back(Node{ 0, 0, 0 });
	nodes.push_back(Node{ 3, -5, 2 });
	nodes.push_back(Node{ 6, 4, 6 });
	nodes.push_back(Node{ -5, -1, 8 });
	nodes.push_back(Node{ 5, 3, 4 });
	nodes.push_back(Node{ -3, 4, 6 });
#else
	float nLocations, temp;

	std::ifstream file;
	char filename[STR_LEN] = ".\\EVRP\\Data_Sets\\EVRP TW\\c101C5.txt";

	file.open(filename);
	if (!file.is_open())
	{
		std::cout << "Failed to open data file, exiting" << std::endl;
		exit(1);
	}
	else
	{

		std::string ID;
		char nodeType;
		std::string line;
		double x, y;
		int demand;
		int index = 0;
		data.customerStartIndex = -1;
		while (std::getline(file, line))
		{
			std::istringstream iss(line);
			if (!(iss >> ID >> nodeType >> x >> y >> demand))
			{
				char type = line[0];
				int pos = 0;
				std::string token;
				while ((pos = line.find('/')) != std::string::npos)
				{
					token = line.substr(0, pos);
					line.erase(0, pos + 1);
				}
				if (!token.empty())
				{
					float num = std::stof(token);
					switch (type)
					{
					case 'Q':
						vehicleBatteryCapacity = num;
						break;
					case 'C':
						vehicleLoadCapacity = num;
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
					//std::cout << "Undefined type: " << nodeType << std::endl;
					break;
				}
				nodes.push_back(n);
				index++;
			}


			data = EVRP_Data{ nodes, vehicleBatteryCapacity, vehicleLoadCapacity, vehicleFuelConsumptionRate, data.customerStartIndex };
		}
	}
	file.close();
#endif

	int tot_demand = 0;
	for (const Node& node : nodes)
	{
		tot_demand += node.demand;
	}
	std::cout << "The minimum number of subtours is: " << std::ceil(double(tot_demand) / vehicleLoadCapacity) << std::endl;
}

EVRP_Solver::~EVRP_Solver()
{
}

std::vector<int> EVRP_Solver::SolveEVRP()
{
	GAOptimizer* ga = new GAOptimizer();
	std::vector<int> optimalTour;
	int bestFitness;
	double bestDistance;
	ga->Optimize(data, optimalTour, bestFitness, bestDistance);
	std::cout << "There are " << bestFitness << " subtours in this route, with a total distance of this route is: " << bestDistance << std::endl;
	return optimalTour;
}
