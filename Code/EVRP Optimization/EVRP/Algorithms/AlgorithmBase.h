#pragma once
#include "../GraphStructure.h"
#include "../Vehicle.h"

class AlgorithmBase
{
public:
   AlgorithmBase();
   
   explicit AlgorithmBase(const string &algorithm_name, const EVRP_Data &data)
   {
      problem_data = data;
      vehicle = new Vehicle(data.nodes, data.fuelCapacity, data.loadCapacity, data.fuelConsumptionRate);

      name = algorithm_name;
      hyper_parameters.clear();
   }

   virtual ~AlgorithmBase()
   {
      free(vehicle);
   }
   virtual void Optimize(vector<int>& bestTour, float& bestDistance) = 0;
   string GetName() { return name; }
   vector<string> GetHyperParameters() { return hyper_parameters; }
   
protected:
   EVRP_Data problem_data;
   Vehicle *vehicle;

   
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