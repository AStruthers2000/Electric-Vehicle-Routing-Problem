// Honor code: I pledge that I have neither given nor received help from anyone other than the instructor or the TAs for all work components included here. -- Andrew

/*
* Generic starting poing for the code execution. main function creates a new EVRP_Solver object that has the function SolveEVRP()
* The main function also has some execution time functionality to benchmark the time duration of the solution calculations
*/

#include "EVRP_Solver.h"

int main()
{
    EVRP_Solver* solver = new EVRP_Solver();

    auto startTime = std::chrono::high_resolution_clock::now();

    //Actually solve the problem
    std::vector<int> solution = solver->SolveEVRP();

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    std::cout << "Execution time: " << duration/1000.0f << " seconds" << std::endl;

    return 0;
}