/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "search_dijkstra.hpp"
// #include "lattice_xya.hpp"
#include "debug.hpp"
#include <iostream>
#include <string>
#include <unordered_map>

// Function declarations for different test cases
int TestDijkstra();
// int TestDijkstra2();
int TestDijkstraOnHybridGraph();
int TestDijkstraOnDenseGraph();

int main() {
  // Uncomment the desired test function to run it

  // TestDijkstra();
  // TestDijkstra2();
  // TestDijkstraOnHybridGraph();
  TestDijkstraOnDenseGraph();

  return 0;
};

// Test Dijkstra's algorithm on a sparse graph
int TestDijkstra() {
  std::cout << "####### test_dijkstra.cpp - TestDijkstra() Begin #######" << std::endl;

  // Timer to measure execution time
  raplab::SimpleTimer timer;
  timer.Start();

  // Create a sparse graph and add vertices and edges
  raplab::PlannerGraph* g_ptr;
  raplab::SparseGraph g;

  timer.Start();
  g.AddVertex(0);
  g.AddVertex(1);
  g.AddVertex(2);
  g.AddVertex(3);
  g.AddVertex(4);

  // Add directed arcs and undirected edges with weights
  g.AddArc(0, 1, std::vector<double>({11.3}));
  g.AddArc(1, 0, std::vector<double>({0.3}));
  g.AddEdge(1, 2, std::vector<double>({15.5}));
  g.AddArc(2, 3, std::vector<double>({15.5}));
  g.AddEdge(3, 4, std::vector<double>({16}));
  g.AddEdge(4, 1, std::vector<double>({17.6}));
  g.AddEdge(1, 7, std::vector<double>({9.9}));
  g.AddArc(0, 3, std::vector<double>({6}));

  // Set the graph pointer
  g_ptr = &g;

  // Perform Dijkstra's algorithm to find the shortest path from vertex 0 to 4
  auto dijk = raplab::Dijkstra();
  dijk.SetGraphPtr(g_ptr);
  auto p = dijk.PathFinding(0, 4);
  auto d_all = dijk.GetDistAll();

  // Print the path and distances
  for (auto vv : p) {
    std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
  }
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  // Perform exhaustive backward search from vertex 4
  dijk = raplab::Dijkstra();
  dijk.SetGraphPtr(g_ptr);
  dijk.ExhaustiveBackwards(4);
  d_all = dijk.GetDistAll();
  std::cout << "-----------" << std::endl;
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  // Perform exhaustive forward search from vertex 4
  dijk = raplab::Dijkstra();
  dijk.SetGraphPtr(g_ptr);
  dijk.ExhaustiveForwards(4);
  d_all = dijk.GetDistAll();
  std::cout << "-----------" << std::endl;
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  // Print the execution time
  timer.PrintDuration();

  std::cout << "####### test_dijkstra.cpp - TestDijkstra() End #######" << std::endl;

  return 1;
};

// int TestDijkstra2(){

//   std::cout << "####### test_dijkstra.cpp - TestDijkstra2() Begin #######" << std::endl;
//   raplab::SimpleTimer timer;
//   timer.Start();

//   raplab::Graph* g_ptr ;
//   raplab::LatticeXYA g;

//   timer.Start();

//   g.Init(3,3,8);
//   g.SetKNeighbor(3);

//   g_ptr = &g;

//   auto dijk = raplab::Dijkstra();
//   dijk.SetGraphPtr(g_ptr);
//   dijk.PathFinding(0,40);
//   auto d_all = dijk.GetDistAll();
//   auto p = dijk.GetPath(40);
//   for (auto vv : p) {
//     std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
//   }
//   for (size_t jj = 0; jj < d_all.size(); jj++) {
//     std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
//   }

//   dijk = raplab::Dijkstra();
//   dijk.SetGraphPtr(g_ptr);
//   dijk.ExhaustiveBackwards(40);
//   d_all = dijk.GetDistAll();
//   std::cout << "-----------" << std::endl;
//   for (size_t jj = 0; jj < d_all.size(); jj++) {
//     std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
//   }

//   dijk = raplab::Dijkstra();
//   dijk.SetGraphPtr(g_ptr);
//   dijk.ExhaustiveForwards(0);
//   d_all = dijk.GetDistAll();
//   std::cout << "-----------" << std::endl;
//   for (size_t jj = 0; jj < d_all.size(); jj++) {
//     std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
//   }

//   timer.PrintDuration();

//   std::cout << "####### test_dijkstra.cpp - TestDijkstra2() End #######" << std::endl;

//   return 1;
// };


int TestDijkstraOnHybridGraph(){

  std::cout << "####### test_graph.cpp - TestDijkstraOnHybridGraph() Begin #######" << std::endl;

  raplab::SimpleTimer timer;
  timer.Start();

  // Create a sparse graph
  raplab::SparseGraph g;
  timer.Start();
  g.AddVertex(0);
  g.AddVertex(1);
  g.AddVertex(2);
  g.AddVertex(3);
  g.AddVertex(4);

  // Add arcs and edges
  g.AddArc(0, 1, std::vector<double>({11.3}));
  g.AddArc(1, 0, std::vector<double>({0.3}));
  g.AddEdge(1, 2, std::vector<double>({15.5}));
  g.AddArc(2, 3, std::vector<double>({15.5}));
  g.AddEdge(3, 4, std::vector<double>({15.9}));
  g.AddEdge(4, 1, std::vector<double>({17.6}));

  // Create a 2D grid and set occupancy grid
  raplab::Grid2d gg;
  int r = 10, c = 10;
  std::vector<std::vector<double>> occupancy_grid(r, std::vector<double>(c, 0));
  occupancy_grid[0][2] = 1; // Mark a cell as occupied
  gg.SetOccuGridPtr(&occupancy_grid);
  gg.SetCostScaleFactor(10.0);

  // Create a hybrid graph combining sparse graph and grid
  raplab::HybridGraph2d hg;
  hg.AddGrid2d(&gg);
  hg.AddGrid2d(&gg);
  hg.AddSparseGraph(&g);
  hg.AddSparseGraph(&g);

  // Print successors of specific nodes
  std::cout << hg.GetSuccs(4) << std::endl;
  std::cout << hg.GetSuccs(104) << std::endl;
  std::cout << hg.GetSuccs(200) << std::endl;
  std::cout << hg.GetSuccs(205) << std::endl;

  // Add extra edges to the hybrid graph
  hg.AddExtraEdge(10, 104, raplab::InitVecType(1, 100.0));
  hg.AddExtraEdge(104, 10, raplab::InitVecType(1, 100.0));
  hg.AddExtraEdge(193, 200, raplab::InitVecType(1, 100.0));
  hg.AddExtraEdge(204, 207, raplab::InitVecType(1, 100.0));

  // Perform exhaustive backward search from vertex 207
  auto dijk = raplab::Dijkstra();
  dijk.SetGraphPtr(&hg);
  dijk.ExhaustiveBackwards(207);
  auto d_all = dijk.GetDistAll();
  std::cout << "-----------" << std::endl;
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  // Find the shortest path from vertex 0 to 209
  auto p = dijk.PathFinding(0, 209);
  std::cout << " path(0,207) = " << p << std::endl;

  timer.PrintDuration();

  std::cout << "####### test_graph.cpp - TestDijkstraOnHybridGraph() End #######" << std::endl;

  return 1;
};

// Test Dijkstra's algorithm on a dense graph
int TestDijkstraOnDenseGraph() {
  std::cout << "####### test_graph.cpp - TestDijkstraOnDenseGraph() Begin #######" << std::endl;

  raplab::SimpleTimer timer;
  timer.Start();

  // Create a dense graph
  raplab::DenseGraph g;

  timer.Start();
  std::vector<long> sources = {0, 1, 2, 3};
  std::vector<long> targets = {1, 2, 3, 4};
  std::vector<std::vector<double>> costs = {
      {10}, {20}, {30}, {40}};

  // Create the graph from edges
  g.CreateFromEdges(sources, targets, costs);

  // Print the graph structure
  std::cout << g << std::endl;

  // Perform exhaustive backward search from vertex 3
  auto dijk = raplab::Dijkstra();
  dijk.SetGraphPtr(&g);
  dijk.ExhaustiveBackwards(3);
  auto d_all = dijk.GetDistAll();
  std::cout << "----------- d_all:" << std::endl;
  for (auto iter : d_all) {
    std::cout << iter << " " << std::endl;
  }

  // Find the shortest path from vertex 1 to 3
  auto p = dijk.PathFinding(1, 3);
  std::cout << " path(1,3) = " << p << std::endl;

  timer.PrintDuration();

  std::cout << "####### test_graph.cpp - TestDijkstraOnDenseGraph() End #######" << std::endl;

  return 1;
};
