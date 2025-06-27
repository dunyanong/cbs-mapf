#include "movingai_scen_parser.hpp"

#include <string>
#include <unordered_map>

// Determines if a terrain type (represented by a character) is traversable
bool movingai::traversable(char c)
{
  bool res;
  switch (c) {
  case 'S': // Start position
  case 'W': // Water
  case 'T': // Trees
  case '@': // Impassable terrain
  case 'O': // Obstacles
    res = false; // These terrain types are not traversable
    break;
  default: // All other terrain types are traversable
    res = true;
    break;
  }
  return res;
}

// Constructor for gm_parser, initializes the parser with a map file
movingai::gm_parser::gm_parser(const std::string& filename)
{
    std::fstream mapfs(filename.c_str(), std::fstream::in); // Open the map file for reading
    if(!mapfs.is_open()) // Check if the file was successfully opened
    {
        std::cerr << "err; gm_parser::gm_parser "
            "cannot open map file: "<<filename << std::endl;
        exit(1); // Exit if the file cannot be opened
    }

    this->parse_header(mapfs); // Parse the header of the map file
    this->parse_map(mapfs);    // Parse the map data
    mapfs.close();             // Close the file
}

// Destructor for gm_parser
movingai::gm_parser::~gm_parser()
{
}

// Parses the header of the map file
void 
movingai::gm_parser::parse_header(std::fstream& mapfs)
{
    // Read header fields into a map
    std::unordered_map<std::string, std::string> contents;
    for(int i=0; i < 3; i++) // Expecting 3 header fields
    {
        std::string hfield, hvalue;
        mapfs >> hfield; // Read the header field name
        if(mapfs.good())
        {
            mapfs >> hvalue; // Read the header field value
            if(mapfs.good())
            {
                contents[hfield] = hvalue; // Store the field and value in the map
            }
            else
            {
                std::cerr << "err; map load failed. could not read header." << 
                    hfield << std::endl;
                exit(1); // Exit if the header value cannot be read
            }
        }
        else
        {
            std::cerr << "err;  map load failed. format looks wrong."<<std::endl;
            exit(1); // Exit if the header field cannot be read
        }
    }

    // Validate and store the header fields
    this->header_.type_ = contents[std::string("type")];
    if(this->header_.type_.compare("octile") != 0) // Only "octile" type is supported
    {
        std::cerr << "err; map type " << this->header_.type_ << 
            "is unknown. known types: octile "<<std::endl;;
        exit(1);
    }

    this->header_.height_ = atoi(contents[std::string("height")].c_str());
    if(this->header_.height_ == 0) // Validate height
    {
        std::cerr << "err; map file specifies invalid height. " << std::endl;
        exit(1);
    }

    this->header_.width_ = atoi(contents[std::string("width")].c_str());
    if(this->header_.width_ == 0) // Validate width
    {
        std::cerr << "err; map file specifies invalid width. " << std::endl;
        exit(1);
    }
}

// Parses the map data from the map file
void 
movingai::gm_parser::parse_map(std::fstream& mapfs)
{
    std::string hfield;
    mapfs >> hfield; // Read the "map" keyword
    if(hfield.compare("map") != 0) // Ensure the "map" keyword is present
    {
        std::cerr << "err; map load failed. missing 'map' keyword." 
            << std::endl;
    }

    // Read map data character by character
    int index = 0;
    int max_tiles = this->header_.height_ * this->header_.width_; // Total number of tiles
    while(true)
    {
        char c = mapfs.get(); // Get the next character
        if( !mapfs.good() ) // Check for end of file
        {
            break;
        }

        if(c == ' ' || c == '\t' || c == '\n' || c == '\r') // Skip whitespace
        {
            continue;
        }

        if(index >= max_tiles) // Ignore extra tiles beyond the expected count
        {
            index++;
            continue;
        }

        this->map_.push_back(c); // Add the character to the map
        index++;
    }

    if(index != max_tiles) // Validate the number of tiles read
    {
        std::cerr << "err; expected " << max_tiles
            << " tiles; read " << index <<" tiles." << std::endl;
        exit(1);
    }
}

// Constructor for scenario_manager, initializes with version 1
movingai::scenario_manager::scenario_manager() : version_(1)
{
}

// Destructor for scenario_manager, cleans up allocated memory
movingai::scenario_manager::~scenario_manager()
{
    for(unsigned int i=0; i < experiments_.size(); i++) // Delete all experiments
    {
        delete experiments_[i];
    }
    experiments_.clear(); // Clear the experiments vector
}

// Loads a scenario file
void 
movingai::scenario_manager::load_scenario(const std::string& filelocation)
{
    std::ifstream infile;
    infile.open(filelocation.c_str(),std::ios::in); // Open the scenario file

    if(!infile.good()) // Check if the file was successfully opened
    {
        std::cerr << "err; scenario_manager::load_scenario "
        << "Invalid scenario file: "<<filelocation << std::endl;
        infile.close();
        exit(1);
    }

    sfile_ = filelocation; // Store the file location

    // Check if a version number is given
    float version=0;
    std::string first;
    infile >> first; // Read the first word
    if(first != "version") // If no version is specified, assume version 0
    {
        version = 0.0;
        infile.seekg(0,std::ios::beg); // Reset file pointer to the beginning
    }

    infile >> version; // Read the version number
    if(version == 1.0 || version == 0) // Only version 1.0 and 0 are supported
    {
        load_v1_scenario(infile); // Load version 1.0 scenario
    }
    else
    {
        std::cerr << "err; scenario_manager::load_scenario "
            << " scenario has invalid version number. \n";
        infile.close();
        exit(1);
    }

    infile.close(); // Close the file
}

// Loads a version 1.0 scenario
void 
movingai::scenario_manager::load_v1_scenario(std::ifstream& infile)
{
    int sizeX = 0, sizeY = 0; // Map dimensions
    int bucket; // Experiment ID
    std::string map; // Map file name
    int xs, ys, xg, yg; // Start and goal coordinates
    std::string dist; // Distance between start and goal

    // Read experiments from the file
    while(infile>>bucket>>map>>sizeX>>sizeY>>xs>>ys>>xg>>yg>>dist)
    {
        double dbl_dist = strtod(dist.c_str(),0); // Convert distance to double
        experiments_.push_back(
                new experiment(xs,ys,xg,yg,sizeX,sizeY,dbl_dist,map)); // Create a new experiment

        int precision = 0;
        if(dist.find(".") != std::string::npos) // Determine the precision of the distance
        {
            precision = dist.size() - (dist.find(".")+1);
        }
        experiments_.back()->set_precision(precision); // Set the precision for the experiment
    }
}
