#include "../include/GraphGenerator.h"
#include "../include/Graph.h"
#include "../include/WAGraphGenerator.h"
#include "../include/WCGraphGenerator.h"
#include "../include/json.hpp"
#include "../include/AdjMatrixGraphGenerator.h"

#include <vector>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <functional>
#include <numeric>
#include <fstream>

using json = nlohmann::json;

void displayCycles(Graph &g, std::vector<std::vector<int>> cycles)
{
	if (cycles.size() == 0) return;

	for (const auto& cycle : cycles)
	{
		if (cycle.size() == 0) continue;
		for (size_t i=0; i<cycle.size()-1; i++)
			std::cout << cycle[i] << " --" << g.getWeight(cycle[i], cycle[i+1]) << "-> ";
		
		std::cout << cycle[cycle.size()-1];
		std::cout << "\n";
	}
}

//------------------------------------------------------------------------------------------------

int time(std::function<std::vector<std::vector<int>>(std::vector<std::pair<int, int>>)> algorithm, std::vector<std::pair<int,int>> mandatoryEdges = {})
{
	auto t1 = std::chrono::high_resolution_clock::now();
    volatile auto result = algorithm(mandatoryEdges);
    auto t2 = std::chrono::high_resolution_clock::now();

	int time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
	return time;
}

//------------------------------------------------------------------------------------------------

void tests(int numberOfTests, int minGraphSize, int maxGraphSize, int graphSizeStep, int minHamiltonianCycles, int maxWeight, double probabilityOfEdge,  bool output = false) // output = 1 saves to file
{
    std::vector<int> graphSizes;
	std::vector<int> bruteForceAverages;
	std::vector<int> greedyAverages;
	std::vector<int> backtrackingAverages;
	std::vector<int> antColonyAverages;

	for (int currGraphSize = minGraphSize; currGraphSize<=maxGraphSize; currGraphSize += graphSizeStep)
	{
		std::vector<int> bruteForceTimes;
		std::vector<int> greedyTimes;
		std::vector<int> backtrackingTimes;
		std::vector<int> antColonyTimes;

		for (int test = 1; test <= numberOfTests; test++)
		{
			Graph g;
			AdjMatrixGraphGenerator gen(currGraphSize, probabilityOfEdge, maxWeight);
			g = gen.generate();

			std::vector<std::pair<int,int>> mandatoryEdges;
			mandatoryEdges.push_back(g.chooseRandomEdge());

			std::function<std::vector<std::vector<int>>(std::vector<std::pair<int, int>>)> boundBruteForceBTSP = std::bind(&Graph::bruteForceBTSP, &g, std::placeholders::_1);
			std::function<std::vector<std::vector<int>>(std::vector<std::pair<int, int>>)> boundGreedyBTSP = std::bind(&Graph::greedyBTSP, &g, std::placeholders::_1);
			std::function<std::vector<std::vector<int>>(std::vector<std::pair<int, int>>)> boundBacktrackingBTSP = std::bind(&Graph::backtrackingBTSP, &g, std::placeholders::_1);
			std::function<std::vector<std::vector<int>>(std::vector<std::pair<int, int>>)> boundAntColonyBTSP = std::bind(&Graph::antColonyBTSP, &g, std::placeholders::_1);
	
			if (currGraphSize <= 11)
			{
				bruteForceTimes.push_back(time(boundBruteForceBTSP, mandatoryEdges));
			}
			greedyTimes.push_back(time(boundGreedyBTSP, mandatoryEdges));
			backtrackingTimes.push_back(time(boundBacktrackingBTSP, mandatoryEdges));
			antColonyTimes.push_back(time(boundAntColonyBTSP, mandatoryEdges));


			if (!bruteForceTimes.empty() && bruteForceTimes[bruteForceTimes.size() - 1] == 0)
			{
				test--;
				bruteForceTimes.pop_back();
				greedyTimes.pop_back();
				antColonyTimes.pop_back();

				if (currGraphSize <= 11)
					backtrackingTimes.pop_back();
			}
		}


		graphSizes.push_back(currGraphSize);
		if (currGraphSize <= 11)
			bruteForceAverages.push_back(std::accumulate(bruteForceTimes.begin(), bruteForceTimes.end(), 0)/bruteForceTimes.size());
		else
			bruteForceAverages.push_back(1);
		greedyAverages.push_back(std::accumulate(greedyTimes.begin(), greedyTimes.end(), 0)/greedyTimes.size());
		antColonyAverages.push_back(std::accumulate(antColonyTimes.begin(), antColonyTimes.end(), 0)/antColonyTimes.size());
		backtrackingAverages.push_back(std::accumulate(backtrackingTimes.begin(), backtrackingTimes.end(), 0)/backtrackingTimes.size());
	}


	switch(output)
	{
		case false:
		{
			std::cout << std::setw(15) << "Graph Size" << 
						 std::setw(15) << "bruteForce" << 
						 std::setw(15) << "greedy" << 
						 std::setw(15) << "backtracking" <<
						 std::setw(15) << "antColony" << "\n";
			for (size_t i = 0; i < graphSizes.size(); i++)
			{
				std::cout << std::setw(15) << graphSizes[i] << 
							 std::setw(15) << bruteForceAverages[i] << 
							 std::setw(15) << greedyAverages[i] << 
							 std::setw(15) << backtrackingAverages[i] << 
							 std::setw(15) << antColonyAverages[i] << "\n";
			}
			break;
		}
		case true:
		{
			json result;
			result["graphSizes"] = graphSizes;
			result["bruteForceAverages"] = bruteForceAverages;
			result["greedyAverages"] = greedyAverages;
			result["backtrackingAverages"] = backtrackingAverages;
			result["antColonyAverages"] = antColonyAverages;


			std::ofstream file("algorithm_averages.json");
			file << result.dump(4);
			file.close();

			std::cout << "JSON file 'algorithm_averages.json' has been generated successfully." << std::endl;
			break;
		}
	}
}

//------------------------------------------------------------------------------------------------

int main() {
	int maxWeight = 10;
	bool directed = false;
	bool weight = true;

	int numberOfTestsPerGraphSize = 20;
	double probability = 0.8;
	int minGraphSize = 5;
	int maxGraphSize = 50;
	int step = 1;
	tests(numberOfTestsPerGraphSize, 
		  minGraphSize, 
		  maxGraphSize, 
		  step, 
		  1,
		  maxWeight,
		  probability,
		  false);

	// Graph g;
	// // AdjMatrixGraphGenerator gen(minGraphSize, probability, maxWeight);
	// WCGraphGenerator gen(minGraphSize);
	// gen.setMaxWeight(10);
	// g = gen.generate();

	// g.display();
	// std::vector<std::vector<int>> cycles = g.backtrackingBTSP();
	// displayCycles(g, cycles);

	return 0;
}