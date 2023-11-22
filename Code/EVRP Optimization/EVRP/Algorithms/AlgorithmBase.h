#pragma once
#include <iostream>
#include "../Vehicle.h"
#include "../SolutionSet.h"

class AlgorithmBase
{
public:
   AlgorithmBase();
   
   explicit AlgorithmBase(const string &algorithm_name, const ProblemDefinition* data)
   {
      problem_data = data;
      vehicle = new Vehicle(problem_data);

      name = algorithm_name;
      hyper_parameters.clear();

      found_tours = new SolutionSet();
      
   }

   virtual ~AlgorithmBase()
   {
      free(vehicle);
   }
   virtual void Optimize(solution &best_solution) = 0;
   string GetName() { return name; }
   vector<string> GetHyperParameters() { return hyper_parameters; }
   SolutionSet* GetFoundTours() const { return found_tours; }
   
protected:
   const ProblemDefinition *problem_data;
   Vehicle *vehicle;

   SolutionSet* found_tours;
   
   void SetHyperParameters(const vector<string> &params)
   {
      for(const auto &iter : params)
      {
         hyper_parameters.push_back(iter);
      }
   }

   static void PrintIfTheTimeIsRight(const string &alg, const int &current, const int &max)
   {
      if(current % 10 == 0) cout << alg << " is " << (static_cast<float>(current) / static_cast<float>(max)) * 100 << " percent complete"<<endl; 
   }

private:
   string name;
   vector<string> hyper_parameters;
};
