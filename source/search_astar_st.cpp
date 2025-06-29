/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "search_astar_st.hpp"
#include <debug.hpp>
#include <set>
#include <limits>

namespace raplab{

// Converts the state (v, t) to a string representation for easy identification.
std::string StateST::ToStr() {
  std::string out = std::to_string(v) + "," + std::to_string(t);
  return out;
};

// Equality operator for comparing two states based on vertex and time.
bool StateST::operator==(StateST& another) {
  return (v == another.v) && (t == another.t);
};

// Overloads the stream operator to print the state in a readable format.
std::ostream& operator<<(std::ostream& os, StateST& s) {
  os << "{id:" << s.id << ",v:" << s.v << ",t:" << s.t << "}";
  return os;
};

/////////////////////////////////////

// Default constructor for StateSpaceST.
StateSpaceST::StateSpaceST() {};

// Constructor that accepts a pointer to a Grid2d object.
StateSpaceST::StateSpaceST(Grid2d* ptr) {
  
};

// Destructor for StateSpaceST.
StateSpaceST::~StateSpaceST() {};

// Generates successors for a given state `s` and calculates their costs.
void StateSpaceST::GetSuccs(
  StateST& s, 
  std::vector<StateST>* out, 
  std::vector<std::vector<double>>* out_costs
) {
  std::cout << "[DEBUG] Generating successors for state: " << s << std::endl;

  auto uAll = Grid2d::GetSuccs(s.v);
  uAll.push_back(s.v); // Add the option to wait in place.

  for (auto& u : uAll) {
      int y = _k2r(u);
      int x = _k2c(u);

      if (!IsWithinBorder(y, x)) {
          std::cout << "[DEBUG] Skipping out-of-bounds successor: " << u << std::endl;
          continue;
      }

      if (_occu_grid_ptr->at(y).at(x) != 0) {
          std::cout << "[DEBUG] Skipping occupied successor: " << u << std::endl;
          continue;
      }

      size_t kth = out->size();
      out->resize(kth + 1);
      out->at(kth).v = u;
      out->at(kth).t = s.t + 1;
      out_costs->resize(kth + 1);
      out_costs->at(kth).resize(1);
      out_costs->at(kth)[0] = 1; // Move cost is 1

      std::cout << "[DEBUG] Added successor: " << u << " at time: " << s.t + 1 << std::endl;
  }
}

//////////

// Constructor for AstarSTGrid2d, initializes the class name.
AstarSTGrid2d::AstarSTGrid2d() {
  _class_name = "AstarSTGrid2d";
};

// Destructor for AstarSTGrid2d.
AstarSTGrid2d::~AstarSTGrid2d() {

};

// Adds a node constraint at a specific time `t`.
void AstarSTGrid2d::AddNodeCstr(long nid, long t) {
  if ( _avl_node.find(nid) == _avl_node.end() ) {
    _avl_node[nid] = raplab::AVLTree<long>();
  }
  _avl_node[nid].Add(t); // Add the unsafe interval.

  // Update the latest node constraint time if necessary.
  if (t > _last_nc_t) {
    _last_nc_t = t;
  }
  return;
};

// Adds an edge constraint between two nodes at a specific time `t`.
void AstarSTGrid2d::AddEdgeCstr(long u, long v, long t) {
  if ( _avl_edge.find(u) == _avl_edge.end() ) {
    _avl_edge[u] = std::unordered_map<long, raplab::AVLTree<long> >();
  }
  if ( _avl_edge[u].find(v) == _avl_edge[u].end() ) {
    _avl_edge[u][v] = raplab::AVLTree<long>();
  }
  _avl_edge[u][v].Add(t);
  return;
};

// Main search function for A* algorithm.
int AstarSTGrid2d::_search() {
  
  // Debugging information about the search initialization.
  if (DEBUG_ASTAR_ST){
    std::cout << "[DEBUG] AstarSTGrid2d::_search, _vs = " << _vs << " _vg = " << _vg << " _mode = " 
      << _mode << " _time_limit = " << _time_limit  << " _class_name = " << _class_name << std::endl; 
  }

  // Check if the start vertex exists in the graph.
  if ( !(_graph->HasVertex(_vs)) ) {
    std::cout << "[ERROR] AstarSTGrid2d, input v_start " << _vs << " does not exist !!" << std::endl;
    throw std::runtime_error( "[ERROR] AstarSTGrid2d, input v_start does not exist !!" );
    return -1;
  } 

  // Check if the goal vertex exists in the graph (for mode 0).
  if ( (_mode == 0 ) && !(_graph->HasVertex(_vg)) ) {
    std::cout << "[ERROR] AstarSTGrid2d, input v_goal " << _vg << " does not exist !!" << std::endl;
    throw std::runtime_error( "[ERROR] AstarSTGrid2d, input v_goal does not exist !!" );
    return -2;
  }

  _init_more(); // Additional initialization.

  auto ss = dynamic_cast<StateSpaceST*>(_graph);
  if (!ss) {
      std::cerr << "[ERROR] AstarSTGrid2d::_search: _graph is not a valid StateSpaceST object!" << std::endl;
      std::cerr << "[DEBUG] _graph type: " << typeid(*_graph).name() << std::endl;
      throw std::runtime_error("[ERROR] Invalid StateSpaceST object.");
  }

  // ### Initialization ###
  SimpleTimer timer;
  timer.Start();

  // Initialize the start state.
  StateST s0;
  s0.v = _vs;
  s0.t = _ts;
  s0.id = _gen_label_id();
  _states[s0.id] = s0; // Store the state by its ID.
  _g_all[s0.ToStr()] = 0; // Cost to reach the start state is 0.
  _parent[s0.id] = -1; 
  _reached_goal_state_id = -1;

  // Push the start state into the open list.
  _open.push( Node(s0.v, 0, _heuristic(s0.v)) );

  long n_exp = 0; // Number of expanded nodes.
  long n_gen = 0; // Number of generated nodes.

  // 
  std::unordered_set<long> visited; // Track visited states

  while (!_open.empty()) {
    std::cout << "[DEBUG] Open list size: " << _open.size() << std::endl;

    // Check for timeout
    if (timer.GetDurationSecond() > _time_limit) {
        std::cout << "[INFO] AstarSTGrid2d::Search timeout!" << std::endl;
        break;
    }

    auto cur = _open.top();
    _open.pop();

    if (visited.count(cur.id)) {
        continue; // Skip already visited states
    }
    visited.insert(cur.id);

    // Process the current state
    StateST s = _states[cur.id];
    double g_s = _g_all[s.ToStr()];

    std::cout << "[DEBUG] Processing state: " << s << " with g=" << g_s << std::endl;

    // Check if the goal condition is met
    if (_check_terminate(s)) {
        std::cout << "[DEBUG] Goal reached at state: " << s << std::endl;
        _reached_goal_state_id = s.id;
        break;
    }

    // Expand the current state
    std::vector<StateST> succs;
    std::vector<std::vector<double>> cvecs;
    ss->GetSuccs(s, &succs, &cvecs);

    std::cout << "[DEBUG] Number of successors: " << succs.size() << std::endl;

    for (int idx = 0; idx < succs.size(); idx++) {
        auto& s2 = succs[idx];

        // Check for collisions
        if (_collide_check(s.v, s2.v, s.t)) {
            std::cout << "[DEBUG] Collision detected for successor: " << s2 << std::endl;
            continue;
        }

        double g2 = g_s + cvecs[idx][0];
        auto s2str = s2.ToStr();

        if (_g_all.find(s2str) != _g_all.end() && _g_all[s2str] < g2) {
            std::cout << "[DEBUG] Pruning successor: " << s2 << " with g=" << g2 << std::endl;
            continue;
        }

        // Add the successor to the open list
        s2.id = _gen_label_id();
        _states[s2.id] = s2;
        _parent[s2.id] = s.id;
        _g_all[s2str] = g2;

        auto h2 = _wh * _heuristic(s2.v);
        _open.push(Node(s2.v, g2, h2));

        std::cout << "[DEBUG] Added successor: " << s2 << " with g=" << g2 << " and h=" << h2 << std::endl;
    }
}  
  std::cout << "[INFO] AstarSTGrid2d::Search exit after " << timer.GetDurationSecond() << " seconds with n_exp=" << n_exp << " n_gen=" << n_gen << std::endl;

  return 1;
};

// Checks if the goal condition is met.
bool AstarSTGrid2d::_check_terminate(StateST& s) {
  if ((s.v == _vg) && (s.t > _last_nc_t)) {
      std::cout << "[DEBUG] Goal condition met for state: " << s << std::endl;
      return true;
  }
  return false;
}

// Generates a unique label ID for states.
long AstarSTGrid2d::_gen_label_id() {

  long out = _label_id_gen++;

  // Resize internal storage if necessary.
  if (_label_id_gen > __vec_alloc_total){
    __vec_alloc_total += __vec_alloc_batch;
    _states.resize(__vec_alloc_total);
    _parent.resize(__vec_alloc_total);
    if (__vec_alloc_batch < __vec_alloc_batch_max){
      __vec_alloc_batch += __vec_alloc_batch;
    }
  }

  return out;
};

// Heuristic function for estimating the cost to the goal.
double AstarSTGrid2d::_heuristic(long v) {

  // --- Dijkstra --- 
  auto ss = dynamic_cast<StateSpaceST*>(_graph);
  auto out = _dijk.GetDistValue( v );
  if (out < 0) {
    throw std::runtime_error( "[ERROR], unavailable heuristic !?" );
  }
  return out;
};

// Additional initialization for the search.
void AstarSTGrid2d::_init_more() {

  AstarGrid2d::_init_more();

  _dijk.SetGraphPtr(_graph);
  _dijk.ExhaustiveBackwards(_vg, std::numeric_limits<double>::infinity(), 0) ;
};

// Checks for collisions (node and edge constraints).
bool AstarSTGrid2d::_collide_check(long v1, long v2, long t) {
  if (_avl_node[v2].Find(t + 1).h != 0) {
      std::cout << "[DEBUG] Node constraint violated at vertex: " << v2 << " at time: " << t + 1 << std::endl;
      return true;
  }

  if (_avl_edge.find(v1) != _avl_edge.end() && 
      _avl_edge[v1].find(v2) != _avl_edge[v1].end() && 
      _avl_edge[v1][v2].Find(t).h != 0) {
      std::cout << "[DEBUG] Edge constraint violated between " << v1 << " and " << v2 << " at time: " << t << std::endl;
      return true;
  }

  return false;
}

// Retrieves the path from the start to the goal.
std::vector<long> AstarSTGrid2d::GetPath(long v, bool do_reverse) {
  if ( (_parent.size() == 0) || (_reached_goal_state_id == -1) ){
    return std::vector<long>() ;
  }
  long sid = _reached_goal_state_id;
  std::vector<long> out;
  out.push_back(_states[sid].v);
  while( _parent[sid] != -1 ) {
    out.push_back(_states[_parent[sid]].v);
    sid = _parent[sid];
  }

  if (do_reverse) {
    std::vector<long> path;
    for (size_t i = 0; i < out.size(); i++) {
      path.push_back(out[out.size()-1-i]);
    }
    return path;
  }else{
    return out;
  }
};

// Runs the A* search algorithm on a given graph.
int RunAstarSTGrid2d(
  PlannerGraph* g, long vo, long vd, double time_limit, 
  std::vector< std::vector<long> >& ncs, std::vector< std::vector<long> >& ecs, std::vector<long>* out_path) 
{
  auto g2 = dynamic_cast<Grid2d*>(g);
  raplab::StateSpaceST ss;
  ss.SetOccuGridPtr(g2->GetOccuGridPtr());

  auto planner = raplab::AstarSTGrid2d();
  planner.SetGraphPtr(&ss);

  // Add node constraints.
  for (auto nc: ncs) {
    planner.AddNodeCstr(nc[0], nc[1]);
  }

  // Add edge constraints.
  for (auto ec: ecs) {
    planner.AddEdgeCstr(ec[0], ec[1], ec[2]);
  }

  // Perform pathfinding.
  *out_path = planner.PathFinding(vo, vd, time_limit);
  return out_path->size();
};

} // end namespace raplab
