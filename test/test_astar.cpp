/*******************************************
 * Author: Zhongqiang Richard Ren.
 * All Rights Reserved.
 *******************************************/

#include "debug.hpp"
#include "movingai_map_parser.hpp"
#include "movingai_scen_parser.hpp"
#include "search_astar.hpp"
#include <cassert>

// Function declarations for testing different A* implementations
int TestAstar();
int TestAstarGrid2d();
int TestMovingAIGrid2D();

int main() {
  // Uncomment the desired test function to run it
  // TestAstar();
  // TestAstarGrid2d();
  TestMovingAIGrid2D();

  return 0;
};

int TestAstar() {
  std::cout << "####### TestAstar() Begin #######" << std::endl;

  // Timer to measure execution time
  raplab::SimpleTimer timer;
  timer.Start();

  // Create a sparse graph and add vertices and edges
  raplab::PlannerGraph *g_ptr;
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

  g_ptr = &g;

  // Initialize A* planner and set the graph
  auto pp = raplab::Astar();
  pp.SetGraphPtr(g_ptr);

  // Perform pathfinding from vertex 0 to vertex 4
  auto p = pp.PathFinding(0, 4);

  // Retrieve and print distances for all vertices
  auto d_all = pp.GetDistAll();
  for (auto vv : p) {
    std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
  }
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  // Print the duration of the operation
  timer.PrintDuration();

  std::cout << "####### TestAstar() End #######" << std::endl;

  return 1;
};

int TestAstarGrid2d() {
  std::cout << "####### TestAstarGrid2d() Begin #######" << std::endl;

  // Timer to measure execution time
  raplab::SimpleTimer timer;
  timer.Start();

  // Create a 2D grid and initialize an occupancy grid
  raplab::Grid2d g;
  std::vector<std::vector<double>> occupancy_grid;
  occupancy_grid.resize(10);
  for (int i = 0; i < 10; i++) {
    occupancy_grid[i].resize(10, 0);
  }

  // Set obstacles in the grid (row 5 is blocked except for one cell)
  for (int i = 0; i < 10; i++) {
    if (i == 5) {
      continue;
    }
    occupancy_grid[5][i] = 1;
  }
  g.SetOccuGridPtr(&occupancy_grid);

  // Initialize A* planner for the grid
  timer.Start();
  auto pp = raplab::AstarGrid2d();
  pp.SetGraphPtr(&g);

  // Perform pathfinding from cell 0 to cell 99
  auto p = pp.PathFinding(0, 99);

  // Retrieve and print distances for all cells
  auto d_all = pp.GetDistAll();
  for (auto vv : p) {
    std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
  }
  for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
  }

  // Print the duration of the operation
  timer.PrintDuration();

  std::cout << "####### TestAstarGrid2d() End #######" << std::endl;

  return 1;
};

int TestMovingAIGrid2D() {
  // Load map and scenario files for the MovingAI benchmark
  std::string mapfile = "./data/arena/arena.map";
  std::string scenfile = "./data/arena/arena.map.scen";
  movingai::gridmap _g(mapfile);
  movingai::scenario_manager scen_mgr;
  scen_mgr.load_scenario(scenfile);

  // Timer to measure execution time
  raplab::SimpleTimer timer;

  // Create a 2D grid and initialize an occupancy grid based on the map
  raplab::Grid2d g;
  std::vector<std::vector<double>> occupancy_grid;
  int maxh = _g.height_, maxw = _g.width_;
  occupancy_grid.resize(maxh);
  for (int i = 0; i < maxw; i++)
    occupancy_grid[i].resize(maxw, 0);

  // Mark obstacles in the grid
  for (int y = 0; y < maxh; y++)
    for (int x = 0; x < maxw; x++) {
      if (_g.is_obstacle({x, y}))
        occupancy_grid[y][x] = 1;
    }
  g.SetOccuGridPtr(&occupancy_grid);

  // Initialize A* and Dijkstra planners for the grid
  auto pp = raplab::AstarGrid2d();
  pp.SetGraphPtr(&g);

  auto dijk = raplab::Dijkstra();
  dijk.SetGraphPtr(&g);

  // Iterate through all experiments in the scenario file
  for (int i = 0; i < scen_mgr.num_experiments(); i++) {
    int sx, sy, gx, gy, sid, gid;
    auto expr = scen_mgr.get_experiment(i);

    // Get start and goal positions and convert them to grid indices
    sx = expr->startx();
    sy = expr->starty();
    sid = sy * maxw + sx;

    gx = expr->goalx();
    gy = expr->goaly();
    gid = gy * maxw + gx;

    // Perform pathfinding using A* and measure the cost and time
    timer.Start();
    auto path = pp.PathFinding(sid, gid);
    auto cost = pp.GetDistValue(path.back());
    auto tcost = timer.GetDurationSecond();

    // Perform pathfinding using Dijkstra for comparison
    auto path_dij = dijk.PathFinding(sid, gid);
    auto cost_dij = dijk.GetDistValue(path_dij.back());

    // Assert that the costs from A* and Dijkstra are nearly equal
    assert(fabs(cost - cost_dij) < 1e-6);

    // Print the results, including reference cost from the scenario
    std::cout << "raplab::AstarGrid2d Cost: " << cost 
              << ", dij cost: " << cost_dij
              << ", ref cost: " << expr->distance() 
              << " took " << tcost << "s" << std::endl;
  }

  return 1;
}
