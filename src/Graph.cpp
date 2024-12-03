
#include "../include/Graph.h"
#include "../include/json.hpp"
#include <iostream>
#include <vector>
#include <limits>
#include <functional>
#include <algorithm>
#include <fstream>
#include <set>
#include <unordered_set>
#include <random>


using json = nlohmann::json;
// using vec = std::vector

//////////////////////////////////////////////////////////////////////////////////////////////////
/// CONSTRUCTORS
//////////////////////////////////////////////////////////////////////////////////////////////////

Graph::Graph() : nodeCount(0), edgeCount(0), directed(false), weighted(false) {}

//------------------------------------------------------------------------------------------------

Graph::Graph(bool directed, bool weighted) : nodeCount(0), edgeCount(0)
{
    this->directed = directed;
    this->weighted = weighted;
    // graph.reserve(n);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/// DESTRUCTOR
//////////////////////////////////////////////////////////////////////////////////////////////////

Graph::~Graph() 
{
    clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/// OPERATIONS
//////////////////////////////////////////////////////////////////////////////////////////////////

bool Graph::isCycleBackToStart (const std::vector<int> &path) 
{
    auto& neighbors = graph[path.back()];
    return std::any_of(neighbors.begin(), neighbors.end(), 
                       [&path](const std::pair<int, int>& neigh) { return neigh.first == path[0]; });
}

//------------------------------------------------------------------------------------------------

int Graph::addNode(const int& node) 
{
    if (graph.find(node) == graph.end()) 
    {
        nodes.push_back(node);
        graph[node] = std::vector<std::pair<int, int>>();
        nodeCount++;
        return 1;
    }
    // std::cerr << "Node already exists\n";
    return -1;
}	

//------------------------------------------------------------------------------------------------

int Graph::addEdge(const int& u, const int& v, int weight)
{
    // if (!weighted)
    //     weight = 1;
    addNode(u);
    addNode(v);

    for (auto const& neigh : graph[u]) 
    {
        if (neigh.first == v)
        {
            // std::cerr << "DUPLICATE EDGES - tried creating second edge between already connected nodes.\n";
            return -1;
        }
    }
    
    graph[u].push_back({v, weight});
    if (!directed)
    {
        graph[v].push_back({u, weight});
    }
    edgeCount++;
    return 1;
}

//------------------------------------------------------------------------------------------------

std::vector<std::pair<int,int>> Graph::at(int node)
{
    return graph.at(node);
}

//------------------------------------------------------------------------------------------------

std::vector<int> Graph::getAdjacentNodes(const int node) const
{
    std::vector<int> result;
    // Loop through each pair in the list and extract the first element (the adjacent node)
    for (const auto& pair : graph.at(node)) {
        result.push_back(pair.first);  // Push the 'first' element of the pair (the adjacent node)
    }
    return result; 
    // return graph.at(node);
}

// std::list<int> Graph::getAdjacentNodes(const int node)
// {
//     return graph[node].first();
// }

//------------------------------------------------------------------------------------------------

std::vector<int> Graph::getNodes()
{
    // if (nodes.size() != graph.size()) {
    //     std::cerr << "Mismatch: nodes.size() (" << nodes.size() 
    //             << ") != graph.size() (" << graph.size() << ")\n";
    // }

    return nodes;
}

//------------------------------------------------------------------------------------------------

bool Graph::contains(int node) const
{
    return (graph.find(node) != graph.end());
}

//------------------------------------------------------------------------------------------------

size_t Graph::getNodeCount() const
{
    return nodeCount;
}

//------------------------------------------------------------------------------------------------

size_t Graph::getEdgeCount() const
{
    return edgeCount;
}

//------------------------------------------------------------------------------------------------

int Graph::getWeight(const int nodeA, const int nodeB) const
{
    auto it = graph.find(nodeA);
    if (it == graph.end())
        return -2;

    for (const auto &neigh : graph.at(nodeA))
        if (neigh.first == nodeB)
            return neigh.second;

    return -1;
}

//------------------------------------------------------------------------------------------------

bool Graph::areNeighbours(const int nodeA, const int nodeB) const
{
    auto it = graph.find(nodeA);
    if (it == graph.end()) 
        return false;

    for (const auto &neigh : graph.at(nodeA))
        if (neigh.first == nodeB)
            return true;

    return false;
}

//------------------------------------------------------------------------------------------------

bool Graph::isWeighted() const
{
    return weighted;
}

//------------------------------------------------------------------------------------------------

bool Graph::isDirected() const
{
    return directed;	
}

//------------------------------------------------------------------------------------------------

bool Graph::isEmpty() const 
{
    return nodeCount == 0;
}

//------------------------------------------------------------------------------------------------

void Graph::clear() 
{
    graph.clear();
    nodeCount = 0;
    edgeCount = 0;
}

//------------------------------------------------------------------------------------------------

void Graph::display() const
{
    std::string pathL;
    std::string pathR;
    if (directed)
    {
        pathL = "--";
        pathR = "->";
    }
    else
    {
        pathL = "<-";
        pathR = "->";
    }

    for (const auto& node : graph) 
    {
        for (const auto& neigh : node.second)
        {
            if (!directed && neigh.first < node.first)
                continue;

            std::cout << node.first << " " << pathL << neigh.second << pathR << " " << neigh.first << "\n";
        }
    }			
}

//------------------------------------------------------------------------------------------------

void Graph::raw_display() const
{
    for (const auto& node : graph) 
    {
        std::cout << node.first << ": ";
        for (const auto& neigh : node.second)
        {
            std::cout << "{" << neigh.first << ", " << neigh.second << "}, ";
        }
        std::cout << "\n";
    }			
}

//------------------------------------------------------------------------------------------------

void Graph::saveToFile(std::string fileName) const
{
    json j_graph;

    // Store graph properties
    j_graph["directed"] = directed;
    j_graph["weighted"] = weighted;
    j_graph["nodeCount"] = nodeCount;
    j_graph["edgeCount"] = edgeCount;

    // Store nodes and edges
    j_graph["nodes"] = nodes;
    j_graph["edges"] = json::array();

    for (const auto& node : graph)
    {
        json j_node;
        j_node["node"] = node.first;
        j_node["adjacent"] = json::array();
        
        for (const auto& neigh : node.second)
        {
            json j_edge;
            j_edge["neighbor"] = neigh.first;
            j_edge["weight"] = neigh.second;
            j_node["adjacent"].push_back(j_edge);
        }
        
        j_graph["edges"].push_back(j_node);
    }

    // Write to file
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << j_graph.dump(4); // Indented output for readability
        file.close();
        std::cout << "Graph saved to " << fileName << std::endl;
    }
    else
    {
        std::cerr << "Could not open file " << fileName << " for writing.\n";
    }
}

//------------------------------------------------------------------------------------------------

void Graph::loadFromFile(std::string fileName)
{
    std::ifstream file(fileName);
    if (!file.is_open())
    {
        std::cerr << "Could not open file " << fileName << " for reading.\n";
        return;
    }

    json j_graph;
    file >> j_graph;
    file.close();

    // Clear the existing graph data
    clear();

    // Load graph properties
    directed = j_graph["directed"].get<bool>();
    weighted = j_graph["weighted"].get<bool>();   

    // Load nodes
    for (int node : j_graph["nodes"].get<std::vector<int>>())
        addNode(node);

    // Load edges
    for (const auto& j_node : j_graph["edges"])
    {
        int node = j_node["node"].get<int>();

        for (const auto& j_edge : j_node["adjacent"])
        {
            int neighbor = j_edge["neighbor"].get<int>();
            int weight = j_edge["weight"].get<int>();
            addEdge(node, neighbor, weight);
        }
    }

    if (nodeCount != j_graph["nodeCount"].get<int>()) exit(1);
    if (edgeCount != j_graph["edgeCount"].get<int>()) exit(1);

    std::cout << "Graph loaded from " << fileName << std::endl;
}

//------------------------------------------------------------------------------------------------

int Graph::chooseRandomVertex()
{
    // Check if the vector is empty
    if (nodes.empty()) {
        std::cerr << "The node list is empty!" << std::endl;
        return -1; // Or handle it as per your needs
    }

    // Use random device and engine to generate random indices
    std::random_device rd;
    std::mt19937 gen(rd()); // Mersenne Twister generator
    std::uniform_int_distribution<> dis(0, nodes.size() - 1); // Random index from 0 to size-1

    int randomIndex = dis(gen);  // Get a random index from the range
    return nodes[randomIndex];   // Return the random node at the chosen index
}

std::pair<int, int> Graph::chooseRandomEdge() {
    if (isEmpty()) {
        std::cerr << "The graph has no vertices!" << std::endl;
        return {-1, -1}; // Or handle it as needed
    }

    // Loop until we find a non-isolated vertex with at least one edge
    int u = chooseRandomVertex();
    std::vector<int> neighbours = getAdjacentNodes(u);
    if (u == -1 || neighbours.empty()) {
        std::cerr << "The chosen vertex has no edges!" << std::endl;
        return {-1, -1};
    }

    // Randomly select an edge from the chosen vertex
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, neighbours.size() - 1);

    int randomEdgeIndex = dis(gen);
    int v = neighbours[randomEdgeIndex];  // Get the neighbor (edge)

    return {u, v};  // Return the edge as a pair (u, v)
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/// ALGORITHMS
//////////////////////////////////////////////////////////////////////////////////////////////////

bool Graph::isHamiltonianSafe(int node, std::vector<int>& path, int pos)
{
    // Check if there is an edge from the last node in the path to `node`
    auto& neighbors = graph[path[pos - 1]];
    if (std::any_of(neighbors.begin(), neighbors.end(), 
                    [node](const std::pair<int, int>& neigh) { return neigh.first == node; }))
    {
        // Check if `node` is not already in the path
        return std::find(path.begin(), path.end(), node) == path.end();
    }
    return false;
}

//------------------------------------------------------------------------------------------------

std::vector<std::vector<int>> Graph::backtrackingHamiltonianCycles()
{   
    hamCycles.clear();
    std::vector<bool> visited(nodeCount+1, false);
    std::vector<int> path;
    int startNode = 0;
    path.push_back(startNode);
    visited[startNode] = true;

    helperFindHamiltonianCycle(path, visited, 1);

    return hamCycles;
}

//------------------------------------------------------------------------------------------------

void Graph::helperFindHamiltonianCycle (std::vector<int>& path, std::vector<bool>& visited, int pos)
{
    if (pos == nodeCount)
    {
        if (isCycleBackToStart(path))
        {
            path.push_back(path[0]);
            hamCycles.push_back(path);
            path.pop_back();
        }
        return;
    }

    for (int node : nodes)
    { 
        if (isHamiltonianSafe(node, path, pos) && !visited[node])
        {
            path.push_back(node);
            visited[node] = true;

            helperFindHamiltonianCycle(path, visited, pos+1);

            path.pop_back();
            visited[node] = false;
        }
    }
}

//------------------------------------------------------------------------------------------------

std::vector<std::vector<int>> Graph::chooseMinimalBottleneckCycles(const std::vector<std::vector<int>>& cycles)
{
    std::vector<std::vector<int>> minimalBottleneckCycles;
    int minMax = INT_MAX;
    for (const auto &cycle : cycles)
    {
        int max = 0;
        for (size_t i=0; i<cycle.size()-1; i++)
            max = std::max(max, getWeight(cycle[i], cycle[i+1]));
        
        if (minMax > max)
        {
            minMax = max;
            minimalBottleneckCycles.clear();
            minimalBottleneckCycles.push_back(cycle);
        }
        else if (minMax == max)
            minimalBottleneckCycles.push_back(cycle);
    }

    return minimalBottleneckCycles;
}

//------------------------------------------------------------------------------------------------

std::vector<std::vector<int>> Graph::selectCyclesConsistingOfChosenEdges(const std::vector<std::vector<int>>& cycles, const std::vector<std::pair<int,int>>& mandatoryEdges)
{
    if (mandatoryEdges.empty())
    {
        return cycles;
    }
    
    std::vector<std::vector<int>> cyclesContainingMandatoryEdges;
	for (std::vector<int> cycle : cycles)
	{
		std::vector<bool> check(cycle.size(), false);
		for (size_t i=0; i<cycle.size()-1; i++)
        {   
			for (size_t i=0; i<check.size(); i++)
				if (std::make_pair(cycle[i], cycle[i+1]) == mandatoryEdges[i])
					check[i] = true;
        }

		if (!std::all_of(check.begin(), check.end(), [](bool v) { return v; }))
            continue;
				
        cyclesContainingMandatoryEdges.push_back(cycle);
	}

	return cyclesContainingMandatoryEdges;
}

//------------------------------------------------------------------------------------------------

std::vector<std::vector<int>> Graph::greedyBTSP(std::vector<std::pair<int,int>> mandatoryEdges)
{
    std::vector<int> cycle;

    for (auto& entry : graph) 
    {
        // Sort the vector of pairs based on the second value of each pair
        std::sort(entry.second.begin(), entry.second.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                return a.second < b.second; 
            });
    }
    
    int nodesLeft = getNodeCount();
    int node = mandatoryEdges.size() > 0 ? mandatoryEdges[0].first : 0;
    if (node < 0 || node >= getNodeCount()) {
        // std::cerr << "Invalid node index: " << node << "\n";
        return {};
    }

    cycle.push_back(node);
    std::vector<bool> visited(getNodeCount(), false);
    visited[node] = true;
    nodesLeft--;
    while(nodesLeft > 0)
    {
        int first = -1;
        for (std::pair<int, int> neigh : graph[node])
        {
            if (visited[neigh.first] == false)
            {
                if (mandatoryEdges.size() > 0) 
                {
                    auto it = std::find(mandatoryEdges.begin(), mandatoryEdges.end(), std::make_pair(node, neigh.first));
                    
                    // If the edge (node, neigh.first) does not exist in mandatoryEdges
                    if (it == mandatoryEdges.end()) 
                    {
                        // Set first to neigh.first and mark it as visited
                        int first = neigh.first;
                        visited[first] = true;

                        // Remove the edges (node, neigh.first) and (neigh.first, node)
                        mandatoryEdges.erase(std::remove(mandatoryEdges.begin(), mandatoryEdges.end(), std::make_pair(node, neigh.first)), mandatoryEdges.end());
                        mandatoryEdges.erase(std::remove(mandatoryEdges.begin(), mandatoryEdges.end(), std::make_pair(neigh.first, node)), mandatoryEdges.end());

                    }
                }

                if (first == -1)
                {
                    first = neigh.first;
                }
            }
        }

        if (first != -1)
        {
            if (visited[first])
            {
                nodesLeft--;
                node = first;
            }
            else
            {
                visited[first] = true;
                nodesLeft--;
                node = first;
            }
        }
        else
        {
            // std::cout << "Couldn't satisfy greedy requirements\n";
            return {std::vector<int>()}; 
        }

        cycle.push_back(node);
    }
    if (getWeight(cycle[0], cycle[cycle.size()-1]))
    {
        cycle.push_back(cycle[0]);
        return {cycle};
    }
    else
    {
        // std::cout << "Couldn't satisfy greedy requirements\n";
        return {std::vector<int>()}; 
    }
}

//------------------------------------------------------------------------------------------------

std::vector<std::vector<int>> Graph::bruteForceHamiltonCycles()
{
    std::vector<std::vector<int>> cycles;
    std::vector<int> nodesPermut = nodes;
    if (nodesPermut.empty())
        return {};

    while (std::next_permutation(nodesPermut.begin()+1, nodesPermut.end()))
    {
        bool valid = true;
        // nodesPermut.push_back(nodesPermut[0]);
        for (size_t i=0; i<nodesPermut.size()-1; i++)
        {
            if (getWeight(nodesPermut[i], nodesPermut[i+1]) <= 0)
            {
                valid = false;
                break;
            }
        }
        if (getWeight(nodesPermut[0], nodesPermut[nodesPermut.size()-1]) <= 0)
            valid = false;

        if (valid)
        {
            std::vector<int> correctPermutation = nodesPermut;
            correctPermutation.push_back(correctPermutation[0]);
            cycles.push_back(correctPermutation);
        }
        // nodesPermut.pop_back();
    }
    // std::cout << "This line makes the code work\n";

    return cycles;
}

//------------------------------------------------------------------------------------------------

std::vector<std::vector<int>> Graph::bruteForceBTSP(std::vector<std::pair<int,int>> mandatoryEdges)
{
    return chooseMinimalBottleneckCycles(selectCyclesConsistingOfChosenEdges(bruteForceHamiltonCycles(), mandatoryEdges));
}

//------------------------------------------------------------------------------------------------

// std::vector<std::vector<int>> Graph::backtrackingBTSP(std::vector<std::pair<int,int>> mandatoryEdges)
// {
//     return chooseMinimalBottleneckCycles(selectCyclesConsistingOfChosenEdges(backtrackingHamiltonianCycles(), mandatoryEdges));
// }

//------------------------------------------------------------------------------------------------

std::vector<std::vector<int>> Graph::backtrackingBTSP(std::vector<std::pair<int,int>> mandatoryEdges)
{
    int startNode;
    if (mandatoryEdges.size() > 0)
        startNode = mandatoryEdges[0].first;
    else
        startNode = 0;

    std::vector<bool> visited(getNodeCount(), false);
    visited[startNode] = true;
    global_max = INT_MAX;
    int maxCurrentPathWeight = 0;

    std::vector<std::vector<int>> results;
    std::vector<int> currentPath = {startNode};
    backtrackingRecursiveExploration(startNode, currentPath, visited, maxCurrentPathWeight, results, mandatoryEdges);

    return results;
}

//------------------------------------------------------------------------------------------------

bool Graph::hasAllMandatoryEdges(const std::vector<int>& currentPath, const std::vector<std::pair<int, int>>& mandatoryEdges) {

    if (mandatoryEdges.empty())
        return true;

    // Create a set to store edges as pairs (sorted to ensure (a,b) and (b,a) are treated the same)
    std::set<std::pair<int, int>> pathEdges;
    
    // Store the adjacent edges from currentPath in the set
    for (size_t i = 0; i < currentPath.size() - 1; ++i) {
        pathEdges.insert({std::min(currentPath[i], currentPath[i+1]), std::max(currentPath[i], currentPath[i+1])});
    }

    // Check if each mandatory edge exists in the set
    for (const auto& edge : mandatoryEdges) {
        if (pathEdges.find({std::min(edge.first, edge.second), std::max(edge.first, edge.second)}) == pathEdges.end()) {
            return false;  // If any mandatory edge is not found, return false
        }
    }

    return true;  // All mandatory edges are found
}

//------------------------------------------------------------------------------------------------

void Graph::backtrackingRecursiveExploration(int currentNode, std::vector<int> currentPath, std::vector<bool> visited, int maxCurrentPathWeight, std::vector<std::vector<int>> &results, const std::vector<std::pair<int,int>> &mandatoryEdges)
{
    if (currentPath.size() == getNodeCount())
    {
        if (getWeight(currentPath[0], currentNode) <= 0)
            return;

        if (!mandatoryEdges.empty() && !hasAllMandatoryEdges(currentPath, mandatoryEdges))
            return;

        if (maxCurrentPathWeight < global_max)
        {
            global_max = maxCurrentPathWeight;
            results.clear();
            currentPath.push_back(currentPath[0]);
            results.push_back(currentPath);
            currentPath.pop_back();
            return;
        }
        else if (maxCurrentPathWeight == global_max)
        {
            currentPath.push_back(currentPath[0]);
            results.push_back(currentPath);
            currentPath.pop_back();
            return;
        }
        else
        {
            return;
        }

    }

    std::vector<std::pair<int,int>> possibleMandatoryEdges;
    for (std::pair<int,int> edge : mandatoryEdges)
    {
        if (currentNode == edge.first || currentNode == edge.second)
        {
            possibleMandatoryEdges.push_back(edge);
        }
    }

    int nextNode = -1;
    switch (possibleMandatoryEdges.size())
    {
        case 2:
        {
            std::pair<int,int> edge = possibleMandatoryEdges[0];
            int nodeA = edge.first;
            int nodeB = edge.second;
            if (!visited[nodeA])
            {
                nextNode = nodeA;
                break;
            }
            if (!visited[nodeB])
            {
                nextNode = nodeB;
                break;
            }

            edge = possibleMandatoryEdges[1];
            nodeA = edge.first;
            nodeB = edge.second;
            if (!visited[nodeA])
            {
                nextNode = nodeA;
                break;
            }
            if (!visited[nodeB])
            {
                nextNode = nodeB;
                break;
            }
            std::cout << "IMPOSSIBLE SCENARIO WRONG MANDATORY EDGES NUMBER\n";
            exit(1);
            break;
        }
        case 1:
        {
            std::pair<int,int> edge = possibleMandatoryEdges[0];
            int u = edge.first;
            int v = edge.second;

            if (!visited[u] || !visited[v])
                nextNode = u == currentNode ? v : u;

            
        }
        case 0:
        {
            std::vector<int> possibleVerticesConnectedToCurrent;
            if (nextNode > 0)
                possibleVerticesConnectedToCurrent = {nextNode};
            else
                possibleVerticesConnectedToCurrent = getAdjacentNodes(currentNode);
            for (int neighbour : possibleVerticesConnectedToCurrent)
            {
                if (!visited[neighbour])
                {
                    int thisEdgeWeight = getWeight(currentNode, neighbour);
                    if (thisEdgeWeight > global_max)
                        continue;

                    visited[neighbour] = true;
                    currentPath.push_back(neighbour);
                    addEdge(currentNode, neighbour, thisEdgeWeight);
                    if (thisEdgeWeight > maxCurrentPathWeight)
                        // maxCurrentPathWeight = thisEdgeWeight;
                        backtrackingRecursiveExploration(neighbour, currentPath, visited, thisEdgeWeight, results, mandatoryEdges);
                    else
                        backtrackingRecursiveExploration(neighbour, currentPath, visited, maxCurrentPathWeight, results, mandatoryEdges);

                    visited[neighbour] = false;
                    currentPath.pop_back();

                }
            }
            break;
        }   
        default:
        {
            std::cout << "IMPOSSIBLE SCENARIO" << std::endl;
            exit(0);
        }
    }
}
