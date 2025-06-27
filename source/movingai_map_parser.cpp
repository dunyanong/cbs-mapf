#include "movingai_map_parser.hpp" // Include the header file for map parsing
#include "movingai_scen_parser.hpp" // Include the header file for scenario parsing

using namespace movingai; // Use the movingai namespace for convenience

// Constructor for gridmap that initializes a gridmap with given height (h) and width (w)
gridmap::gridmap(vid h, vid w) {
  this->db.resize(h * w); // Resize the internal database to hold h * w elements
  fill(this->db.begin(), this->db.end(), false); // Initialize all elements to 'false' (non-traversable)
};

// Constructor for gridmap that initializes a gridmap from a file
gridmap::gridmap(const string &filename) {
  gm_parser parser(filename); // Create a parser object to read the file
  this->height_ = parser.get_header().height_; // Extract and set the height from the file header
  this->width_ = parser.get_header().width_; // Extract and set the width from the file header
  this->db.resize(this->height_ * this->width_); // Resize the internal database to match the map dimensions
  for (vid i = 0; i < this->db.size(); i++) { // Loop through each tile in the map
    auto c = parser.get_tile_at(i); // Get the character representing the tile at index i
    this->db[i] = traversable(c) ? 0 : 1; // Mark the tile as traversable (0) or non-traversable (1)
  }
}
