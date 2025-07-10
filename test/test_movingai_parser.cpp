#include "movingai_map_parser.hpp" // Include the header for parsing map files
#include "movingai_scen_parser.hpp" // Include the header for parsing scenario files
#include <string> // Include the string library for handling strings
#include <iostream> // Include the iostream library for input/output operations
using namespace std; // Use the standard namespace to simplify code

// Function to test parsing of a map file
void test_graph_parser(string filename) {
  // Create a gridmap object by loading the map file
  movingai::gridmap g(filename);
  
  // Print the name of the map file being parsed
  cout << "[Parse map file: " << filename << "]" << endl;
  
  // Print the height and width of the map
  cout << "height: " << g.height_ << endl;
  cout << "width: " << g.width_ << endl;
  
  // Iterate through each cell in the map and print whether it is an obstacle
  for (int y = 0; y < g.height_; y++) {
    for (int x = 0; x < g.width_; x++) 
      cout << g.is_obstacle({x, y}); // Check if the cell at (x, y) is an obstacle
    cout << endl; // Move to the next line after printing a row
  }
}

// Function to test parsing of a scenario file
void test_scen_parser(string filename) {
  // Create a scenario manager object to handle the scenario file
  movingai::scenario_manager scenmgr;
  
  // Load the scenario file
  scenmgr.load_scenario(filename);

  // Print the name of the scenario file being parsed
  cout << "[Parse scen file: " << filename << "]" << endl;
  
  // Iterate through all experiments in the scenario file
  for (int i = 0; i < scenmgr.num_experiments(); i++) {
    // Get the i-th experiment
    auto expr = scenmgr.get_experiment(i);
    
    // Print the details of the experiment: map dimensions, start and goal coordinates, and distance
    cout << expr->mapheight() << " " << expr->mapwidth() << " "
         << expr->startx() << " " << expr->starty() << " "
         << expr->goalx() << " " << expr->goaly() << " "
         << expr->distance() << std::endl;
  }
}

// Main function to test the map and scenario parsers
int main() {
  // Define the path to the map file
  string mapfile = "/Users/ongdunyan/Downloads/LocalCodes/cbs-mapf/data/arena/arena.map";
  
  // Test the map parser with the specified map file
  test_graph_parser(mapfile);
  cout << endl; // Print a blank line for separation
  
  // Define the path to the scenario file
  string scenfile = "/Users/ongdunyan/Downloads/LocalCodes/cbs-mapf/data/arena/arena.map.scen";
  
  // Test the scenario parser with the specified scenario file
  test_scen_parser(scenfile);
}
