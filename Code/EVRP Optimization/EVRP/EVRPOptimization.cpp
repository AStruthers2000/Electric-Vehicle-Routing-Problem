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
* The file EVRPOptimization.cpp contains a generic main function that instantiates an 
* EVRP_Solver object, which has the method SolveEVRP()
*/

#include "EVRP_Solver.h"

#include <thread>

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
    //list of files to run our tests on
    const vector<string> files = {"r101_21.txt", "r201_21.txt", "r202c5.txt", "r202c15.txt",
                            "rc101_21.txt", "rc102c10.txt", "rc103c15.txt", "rc108c5.txt", "rc201_21.txt", "rc201c10.txt", "rc202c15.txt", "rc204c5.txt"};
    
    //iterate through each of the files, so this can run overnight
    for(const auto &file : files)
    {
        //create EVRP_Solver instance and read the file
        //EVRP_Solver::IsGoodOpen() is false if the constructor failed to read the file
        const auto *solver = new EVRP_Solver(file);
        if(solver->IsGoodOpen())
        {
            //What time is it before solving the problem
            //TODO: this timing only works in the single threaded case, figure out how to time each individual thread
            const auto start_time = std::chrono::high_resolution_clock::now();

            //Create 40 threads that each run a call to the solve function
            vector<thread> solver_threads;
            for(size_t i = 0; i < 40; i++)
            {
                //Actually solve the problem
                solver_threads.emplace_back(&EVRP_Solver::SolveEVRP, solver);
            }

            //wait for all threads to finish computing
            for(auto &t : solver_threads)
            {
                t.join();
            }

            //What time is it now that we've solved the problem
            //TODO: see todo above
            const auto end_time = chrono::high_resolution_clock::now();

            //Get the execution time in milliseconds 
            const auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
            cout << "Total execution time: " << static_cast<float>(duration)/1000.0f << " seconds" << endl;
        }
        else
        {
            cout << "EVRP_Solver had problems opening file " << file << ", so we are skipping" << endl;
        }
    }
    return 0;
}