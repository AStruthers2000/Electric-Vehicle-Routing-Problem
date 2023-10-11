// Honor code: I pledge that I have neither given nor received help from anyone other than the instructor or the TAs for all work components included here. -- Andrew

/*! \mainpage EVRP Home Page
* \section proj_description Project Description
* Electric vehicles (EVs) have gained significant attention in recent years 
* as a sustainable solution to reduce greenhouse gas emissions and dependence 
* on fossil fuels. Efficient utilization of EVs involves not only developing 
* advanced battery technologies but also optimizing their routing to maximize 
* their range and minimize energy consumption. The *Electric %Vehicle Routing 
* Problem* (EVRP) addresses the challenge of determining the most efficient 
* routes for a fleet of EVs to serve a set of customer locations while 
* considering their charging requirements.

* This project presents the use of genetic algorithms (GAs) to optimize the EVRP. 
* Genetic algorithms are a class of heuristic search algorithms inspired by the 
* process of natural selection and genetics. They offer an effective and flexible 
* approach to solving optimization problems, making them particularly suitable for 
* addressing complex and dynamic routing problems like EVRP.

* The primary objectives of this project are to describe the EVRP, establish 
* benchmarking criteria, and evaluate the performance of genetic algorithms 
* in comparison to other state-of-the-art methods. By applying GAs to the EVRP, 
* we aim to achieve optimal routing solutions that consider both the customers'
* demands and the limited range of EVs.

* The primary source of information I used for this project came from __The 
* Electric %Vehicle Routing Problem and its Variations: A Literature Review__, 
* published in *Computers & Industrial Engineering* by Ilker Kucukoglu, 
* Reginald Dewil, and Dirk Cattrysse. This source compiles 136 published 
* papers that consider the routing of battery electric vehicles. We will 
* also be using the same dataset as the one discussed.

* \section start Where to Start
* The file EVRPOptimization.cpp contains a generic main function that instanciates an 
* EVRP_Solver object, which has the method SolveEVRP()
*/

#include "EVRP_Solver.h"

/***************************************************************************//**
 * Generic starting point for the code execution. 
 *
 * The main function creates a new EVRP_Solver object that has the function SolveEVRP().
 * The main function also has some execution time functionality to benchmark the time 
 * duration of the solution calculations. 
 * 
 * We create an EVRP_Solver object. The EVRP_Solver constructor reads the data from
 * a file. We then start a timer to measure the execution time of the SolveEVRP function
 * of the EVRP_Solver class.
 ******************************************************************************/
int main()
{
    EVRP_Solver* solver = new EVRP_Solver();

    //What time is it before solving the problem
    auto startTime = chrono::high_resolution_clock::now();

    //Actually solve the problem
    vector<int> solution = solver->SolveEVRP();

    //What time is it now that we've solved the problem
    auto endTime = chrono::high_resolution_clock::now();

    //Get the execution time in milliseconds 
    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
    cout << "Execution time: " << duration/1000.0f << " seconds" << endl;

    return 0;
}