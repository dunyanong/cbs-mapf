/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "search_astar.hpp"
#include <set>
#include <limits>

namespace raplab{

// Constructor for the Astar class
Astar::Astar() {
  _class_name = "Astar"; // Set the class name for identification
};

// Destructor for the Astar class
Astar::~Astar() {

};

// Heuristic function for Astar (default implementation returns 0)
// This can be overridden in derived classes for specific heuristics
double Astar::_heuristic(long v) {
  return 0;
};

// Adds a node to the open set with its distance and heuristic value
void Astar::_add_open(long u, double dist_u) {
  double h =  _wh * _heuristic(u); // Calculate heuristic value with weight
  _open.push(Node(u, dist_u, h)); // Push the node into the priority queue
};

// Sets the weight for the heuristic function
void Astar::SetHeuWeight(double w) {
  if (w < 1.0){ // Ensure the weight is at least 1
    std::cout << "[ERROR] Astar::SetHeuWeight w = " << w << " < 1 !!!" << std::endl;
    throw std::runtime_error("[ERROR]"); // Throw an error if the weight is invalid
  }
  _wh = w; // Set the heuristic weight
};


////////////////////////////


// Constructor for the AstarGrid2d class
AstarGrid2d::AstarGrid2d() {
  _class_name = "AstarGrid2d"; // Set the class name for identification
};

// Destructor for the AstarGrid2d class
AstarGrid2d::~AstarGrid2d() {

};

// Heuristic function for AstarGrid2d
// Calculates Manhattan distance between the current node and the goal
double AstarGrid2d::_heuristic(long v) {
  auto gg = dynamic_cast<Grid2d*>( _graph ); // Cast the graph to a Grid2d type
  auto r = gg->_k2r(v); // Get the row index of the current node
  auto c = gg->_k2c(v); // Get the column index of the current node
  return abs(r - _vd_r) + abs(c - _vd_c); // Return Manhattan distance
};

// Additional initialization specific to AstarGrid2d
void AstarGrid2d::_init_more() {
  auto gg = dynamic_cast<Grid2d*>( _graph ); // Cast the graph to a Grid2d type
  _vd_r = gg->_k2r(_vg); // Get the row index of the goal node
  _vd_c = gg->_k2c(_vg); // Get the column index of the goal node
};

} // end namespace raplab
