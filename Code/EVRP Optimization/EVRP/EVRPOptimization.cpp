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

#include <iostream>
#include <thread>
#include "EVRP_Solver.h"

using namespace std;

/*
//ALL FILES IN ASCENDING ORDER
const vector<string> files = {
 "c101C5.txt", "c103C5.txt",
 "c206C5.txt", "c208C5.txt",
 "r104C5.txt", "r105C5.txt",
 "r202C5.txt", "r203C5.txt",
 "rc105C5.txt", "rc108C5.txt",
 "rc204C5.txt", "rc208C5.txt",

 "c101C10.txt", "c104C10.txt",
 "c202C10.txt", "c205C10.txt",
 "r102C10.txt", "r103C10.txt",
 "r201C10.txt", "r203C10.txt",
 "rc102C10.txt", "rc108C10.txt",
 "rc201C10.txt", "rc205C10.txt",

 "c103C15.txt", "c106C15.txt",
 "c202C15.txt", "c208C15.txt",
 "r102C15.txt", "r105C15.txt",
 "r202C15.txt", "r209C15.txt",
 "rc103C15.txt", "rc108C15.txt",
 "rc202C15.txt", "rc204C15.txt",

 "c101_21.txt", "c102_21.txt", "c103_21.txt", "c104_21.txt", "c105_21.txt", "c106_21.txt", "c107_21.txt", "c108_21.txt", "c109_21.txt", 
 "c201_21.txt", "c202_21.txt", "c203_21.txt", "c204_21.txt", "c205_21.txt", "c206_21.txt", "c207_21.txt", "c208_21.txt", 
 "r101_21.txt", "r102_21.txt", "r103_21.txt", "r104_21.txt", "r105_21.txt", "r106_21.txt", "r107_21.txt", "r108_21.txt", "r109_21.txt", "r110_21.txt", "r111_21.txt", "r112_21.txt", 
 "r201_21.txt", "r202_21.txt", "r203_21.txt", "r204_21.txt", "r205_21.txt", "r206_21.txt", "r207_21.txt", "r208_21.txt", "r209_21.txt", "r210_21.txt", "r211_21.txt", 
 "rc101_21.txt", "rc102_21.txt", "rc103_21.txt", "rc104_21.txt", "rc105_21.txt", "rc106_21.txt", "rc107_21.txt", "rc108_21.txt", 
 "rc201_21.txt", "rc202_21.txt", "rc203_21.txt", "rc204_21.txt", "rc205_21.txt", "rc206_21.txt", "rc207_21.txt", "rc208_21.txt", 
};
*/

enum RunState
{
    Debug,
    Standard_Test,
    Standard_Full,
    Seeded_Test,
    Seeded_Full
};
constexpr RunState State = Seeded_Test;


/**
 * \brief 
 * \param files 
 * \param num_threads 
 * \param function 
 */
void StandardSolve(const vector<string> &files, int num_threads, void(EVRP_Solver::*function)() const)
{
    if(num_threads < 1) num_threads = 1;
    
    //iterate through each of the files, so this can run overnight
    for(const auto &file : files)
    {
        //create EVRP_Solver instance and read the file
        //EVRP_Solver::IsGoodOpen() is false if the constructor failed to read the file
        const auto *solver = new EVRP_Solver(file);
        if(solver->IsGoodOpen())
        {
            //What time is it before solving the problem
            const auto start_time = std::chrono::high_resolution_clock::now();

            //Create n threads that each run a call to the solve function
            vector<thread> solver_threads;
            for(size_t i = 0; static_cast<int>(i) < num_threads; i++)
            {
                //Actually solve the problem
                //solver_threads.emplace_back(&EVRP_Solver::SolveEVRP, solver);
                solver_threads.emplace_back(function, solver);
            }

            //wait for all threads to finish computing
            for(auto &t : solver_threads)
            {
                t.join();
            }
                
            //What time is it now that we've solved the problem and all threads have run
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
}

void SeedSolve(const vector<string> &files, int num_threads, EVRP_Solver::SeedAlgorithm alg)
{
    for(const auto &file : files)
    {
        const auto* solver = new EVRP_Solver(file);
        solver->SolveEVRP_Seed(alg);
    }
}

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
    const vector<string> test_files = { "rc103c15.txt" };

    //ALL NON-TIME-WINDOW-SPECIFIC-PROBLEMS IN ASCENDING ORDER
    const vector<string> full_files = {
        //unique five customer problems
        "c101C5.txt", "c206C5.txt", "r104C5.txt", "r202C5.txt", "rc105C5.txt", "rc204C5.txt", 

        //unique ten customer problems
        "c101C10.txt", "c202C10.txt", "r102C10.txt", "r201C10.txt", "rc102C10.txt", "rc201C10.txt", 

        //unique fifteen customer problems
        "c103C15.txt", "c202C15.txt", "r102C15.txt", "r202C15.txt", "rc103C15.txt", "rc202C15.txt", 

        //unique one hundred customer problems
        "c101_21.txt", "c201_21.txt", "r101_21.txt", "r201_21.txt", "rc101_21.txt", "rc201_21.txt", 
   };
    
    switch(State)
    {
    case Debug:
        {
            const auto *solver = new EVRP_Solver("r101_21.txt");
            if(solver->IsGoodOpen()) solver->DebugEVRP();
            break;
        }
        
    case Standard_Test:
        StandardSolve(test_files, 1, &EVRP_Solver::SolveEVRP);
        break;
        
    case Standard_Full:
        StandardSolve(full_files, 30, &EVRP_Solver::SolveEVRP);
        break;
        
    case Seeded_Test:
        SeedSolve(test_files, 1, EVRP_Solver::NEH);
        break;
        
    case Seeded_Full:
        break;
        
    }
    return 0;
}