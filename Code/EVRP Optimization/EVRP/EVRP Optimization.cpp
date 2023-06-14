// EVRP Optimization.cpp : This file contains the 'main' function. Program execution begins and ends there.
// Honor code: I pledge that I have neither given nor received help from anyone other than the instructor or the TAs for all work components included here. -- Andrew

#include "EVRP_Solver.h"

int main()
{
    EVRP_Solver* solver = new EVRP_Solver();

    auto startTime = std::chrono::high_resolution_clock::now();

    std::vector<int> solution = solver->SolveEVRP();

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    std::cout << "Execution time: " << duration/1000.0f << " seconds" << std::endl;

    return 0;
}