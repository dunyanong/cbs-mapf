/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "mapf_util.hpp"

namespace raplab{

// Overloaded operator<< to print a 2D vector of long integers in a structured format
std::ostream& operator<<(std::ostream& os, const std::vector< std::vector<long> >& ps) {
  os << "{"; // Start of the output
  for (size_t i = 0; i < ps.size(); i++) { // Iterate over each agent's path
    os << "agent:" << i << ",path:{"; // Print agent index and start path
    for (auto& jj : ps[i]) { // Iterate over each step in the agent's path
      os << jj << ","; // Print each step in the path
    }
    os << "};"; // End of the agent's path
  }
  os << "}."; // End of the output
  return os; // Return the output stream
};

// Converts a joint path (2D vector) into a PathSet (pointer to a 2D vector)
void JointPath2PathSet(const std::vector< std::vector<long> >& jp, PathSet* ps) {
  if (jp.size() == 0) { // Check if the input joint path is empty
    std::cout << "[CAVEAT] JointPath2PathSet input joint path has size zero!" << std::endl;
    return; // Exit the function if the input is empty
  }
  ps->clear(); // Clear the existing PathSet
  ps->resize(jp[0].size()); // Resize PathSet to match the number of steps in the joint path
  for (int ri = 0; ri < jp[0].size(); ri++) { // Iterate over each step in the joint path
    (*ps)[ri] = std::vector<long>(); // Initialize a new vector for this step
    (*ps)[ri].resize(jp.size()); // Resize the vector to match the number of agents
    for (size_t j = 0; j < jp.size(); j++) { // Iterate over each agent
      (*ps)[ri][j] = jp[j][ri]; // Assign the agent's position at this step
    }
  }
  return; // Function completed
};

// Constructor for the MAPFPlanner class
MAPFPlanner::MAPFPlanner() {} ;

// Destructor for the MAPFPlanner class
MAPFPlanner::~MAPFPlanner() {} ;

// Sets the graph pointer for the MAPFPlanner and clears start and goal states
void MAPFPlanner::SetGraphPtr(PlannerGraph* g) {
  _graph = g; // Assign the graph pointer
  _starts.clear(); // Clear the start states
  _goals.clear(); // Clear the goal states
};

} // end namespace raplab
