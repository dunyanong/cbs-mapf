/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "search_dijkstra.hpp"
#include <set>
#include <limits>

namespace raplab{

// Constructor for the Dijkstra class
Dijkstra::Dijkstra() {

};

// Destructor for the Dijkstra class
Dijkstra::~Dijkstra() {

};

// Main pathfinding function for Dijkstra's algorithm
// vs: start vertex, vg: goal vertex, time_limit: max time allowed, cdim: cost dimension
std::vector<long> Dijkstra::PathFinding(long vs, long vg, double time_limit, short cdim) {
    _mode = 0; // Mode 0: start-goal pathfinding
    _cdim = cdim;
    _vs = vs;
    _vg = vg;
    _time_limit = time_limit;
    _search(); // Perform the search
    return GetPath(vg); // Return the path to the goal
};

// Exhaustive backwards search from a goal vertex
int Dijkstra::ExhaustiveBackwards(long vg, double time_limit, short cdim) {
    _mode = 1; // Mode 1: exhaustive backwards search
    _cdim = cdim;
    _vs = vg; // Start from the goal vertex
    _time_limit = time_limit;
    _search(); // Perform the search
    return 1; // Success
};

// Exhaustive forwards search from a start vertex
int Dijkstra::ExhaustiveForwards(long vs, double time_limit, short cdim) {
    _mode = 2; // Mode 2: exhaustive forwards search
    _cdim = cdim;
    _vs = vs;
    _time_limit = time_limit;
    _search(); // Perform the search
    return 1; // Success
};

// Core search function for Dijkstra's algorithm
int Dijkstra::_search() {
    if (DEBUG_DIJKSTRA){
        // Debugging information
        std::cout << "[DEBUG] Dijkstra::_search, _vs = " << _vs << " _vg = " << _vg << " _mode = " 
                  << _mode << " _time_limit = " << _time_limit  << " _class_name = " << _class_name << std::endl; 
    }

    // Check if the start vertex exists in the graph
    if ( !(_graph->HasVertex(_vs)) ) {
        std::cout << "[ERROR] Dijkstra, input v_start " << _vs << " does not exist !!" << std::endl;
        throw std::runtime_error( "[ERROR] Dijkstra, input v_start does not exist !!" );
        return -1;
    } 

    // Check if the goal vertex exists in the graph (for mode 0)
    if ( (_mode == 0 ) && !(_graph->HasVertex(_vg)) ) {
        std::cout << "[ERROR] Dijkstra, input v_goal " << _vg << " does not exist !!" << std::endl;
        throw std::runtime_error( "[ERROR] Dijkstra, input v_goal does not exist !!" );
        return -2;
    } 

    auto tstart = std::chrono::steady_clock::now(); // Start timing the search

    // Initialize search data structures
    size_t nV = _graph->NumVertex(); // Number of vertices in the graph
    _v2d.clear(); // Distance vector
    _v2d.resize(nV, std::numeric_limits<double>::infinity()); // Initialize distances to infinity
    _parent.clear(); // Parent vector for path reconstruction
    _parent.resize(nV, -1); // Initialize parents to -1
    _cvec.clear(); // Cost vector
    _cvec.resize(nV);
    while (!_open.empty()) _open.pop(); // Clear the priority queue
    _open.push(Node(_vs)); // Add the start node to the open list
    _v2d[_vs] = 0.0; // Distance to the start node is 0
    _cvec[_vs].resize(_graph->CostDim(), 0); // Initialize the cost vector for the start node

    _init_more(); // Additional initialization (empty for Dijkstra)

    int success = 0; // Success flag

    // Main search loop
    while(!_open.empty()){

        // Check if the time limit has been exceeded
        auto tnow = std::chrono::steady_clock::now();
        if ( std::chrono::duration<double>(tnow-tstart).count() > _time_limit) {
            break; // Timeout
        }

        // Extract the node with the smallest distance from the open list
        auto curr = _open.top(); _open.pop();
        auto v = curr.id;
        if (curr.g > _v2d[v]) { // Skip outdated nodes
            continue;
        }
        if (DEBUG_DIJKSTRA){ std::cout << "[DEBUG] Dijkstra::_search, - popped v = " << v << " g = " << curr.g << std::endl; }

        // Check if the goal has been reached (for mode 0)
        if (_mode == 0 && v == _vg) {
            success = 1;
            break; // Terminate the search
        }

        // Expand the current node
        std::vector<long> nghs; // Neighbors
        std::vector< std::vector<double> > ngh_costs; // Costs to neighbors
        if (_mode == 1) { 
            nghs = _graph->GetPreds(v); // Get predecessors (backwards search)
            ngh_costs = _graph->GetPredCosts(v);
        } else if (_mode == 0 || _mode == 2){
            nghs = _graph->GetSuccs(v); // Get successors (forwards search)
            ngh_costs = _graph->GetSuccCosts(v);
        } else {
            std::cout << "[ERROR] Dijkstra::_search, _mode = " << _mode << std::endl;
            throw std::runtime_error( "[ERROR] Dijkstra::_search, unknown _mode !!" );
        }
        if(DEBUG_DIJKSTRA){std::cout << "[DEBUG] Dijkstra::_search, - get " << nghs.size() << " successors " << std::endl;}
        for (size_t j = 0; j < nghs.size(); j++) {
            // Process each neighbor
            long u = nghs[j];
            auto cvec = ngh_costs[j]; // Cost vector to the neighbor
            auto c = cvec[_cdim]; // Cost in the specified dimension
            if (c < 0){
                // Negative edge cost detected (error in the input graph)
                std::cout << "[ERROR] v = " << v << " u = " << u << " cost_vec(u,v) = " << cvec << " c = " << c << std::endl;
                throw std::runtime_error( "[ERROR] Dijkstra::_search, encounter negative edge costs !!" );
                _v2d.clear();
                return -3; // Failure
            }
            auto dist_u = curr.g + c; // Distance to the neighbor
            auto cvec_u = _cvec[v] + cvec; // Cost vector to the neighbor
            if (DEBUG_DIJKSTRA){ std::cout << "[DEBUG] Dijkstra::_search, --- generate u = " << u << " g = " << dist_u << std::endl; }

            // Update the neighbor if a shorter path is found
            if ( dist_u < _v2d[u] ){
                if (DEBUG_DIJKSTRA){ std::cout << "[DEBUG] Dijkstra::_search, --- g' = " << dist_u << " < g[u]=" << _v2d[u] << ", will update and add u to open..." << std::endl; }
                _v2d[u] = dist_u;
                _cvec[u] = cvec_u;
                _parent[u] = v;
                _add_open(u, dist_u); // Add the neighbor to the open list
            }
        }
    }
  
    return success; // Return success or failure
};

// Add a node to the open list
void Dijkstra::_add_open(long u, double dist_u) {
    _open.push(Node(u, dist_u)); // Re-insert the node
};

// Additional initialization (empty for Dijkstra)
void Dijkstra::_init_more() {
    // Nothing for Dijkstra. Make derived class easier...
};

// Get the path from the start to a given vertex
std::vector<long> Dijkstra::GetPath(long v, bool do_reverse) {
    std::cout << " .s.s.s. GetPath v = " << v << std::endl;
    std::vector<long> out;
    if ((_parent.size() == 0) || (_parent.size() <= v) ){
        return out; // Return an empty path if invalid
    }
    out.push_back(v);
    while( _parent[v] != -1 ) {
        out.push_back(_parent[v]);
        v = _parent[v];
    }

    if (do_reverse) {
        // Reverse the path if requested
        std::vector<long> path;
        for (size_t i = 0; i < out.size(); i++) {
            path.push_back(out[out.size()-1-i]);
        }
        return path;
    } else {
        return out;
    }
};

// Get the distances to all vertices
std::vector<double> Dijkstra::GetDistAll() {
    return _v2d;
};

// Get the distance to a specific vertex
double Dijkstra::GetDistValue(long v) {
    return _v2d[v];
};

// Get the cost vector of the solution path to the goal
std::vector<double> Dijkstra::GetSolutionCost() {
    return _cvec[_vg];
};

// Get the cost vector of the path to a specific vertex
std::vector<double> Dijkstra::GetPathCost(long v) {
    return _cvec[v];
};

} // end namespace raplab
