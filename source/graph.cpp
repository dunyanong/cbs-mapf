/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

 #include "graph.hpp"
 #include "vec_type.hpp"
 
 namespace raplab{
 
 // ############################################################
 // ### SparseGraph Implementation #############################
 // ############################################################
 
 // Constructor
 SparseGraph::SparseGraph() {};
 
 // Destructor
 SparseGraph::~SparseGraph() {};
 
 // Check if vertex exists in graph
 bool SparseGraph::HasVertex(long v) {
   return v < _to.size();
 };
 
 // Check if there's an arc from v to u
 bool SparseGraph::HasArc(long v, long u) {
   for (auto k : _to[v]){
     if (k == u) {return true;}
   }
   return false;
 };
 
 // Get successor vertices of v
 std::vector<long> SparseGraph::GetSuccs(long v) {
   if (!HasVertex(v)) {return std::vector<long>(); }
   return _to[v];
 };
 
 // Get predecessor vertices of v
 std::vector<long> SparseGraph::GetPreds(long v) {
   if (!HasVertex(v)) {return std::vector<long>(); }
   return _from[v];
 };
 
 // Get cost vector for arc from u to v
 CostVec SparseGraph::GetCost(long u, long v) {
   for (size_t idx = 0; idx < _to_cost[u].size(); idx++){
     if (_to[u][idx] == v) {
       return _to_cost[u][idx];
     }
   }
   return std::vector<double>();
 };
 
 // Get all successor costs for vertex u
 std::vector< CostVec > SparseGraph::GetSuccCosts(long u) {
   if (!HasVertex(u)) {return std::vector<std::vector<double>>(); }
   return _to_cost[u];
 };
 
 // Get all predecessor costs for vertex u
 std::vector< CostVec > SparseGraph::GetPredCosts(long u) {
   if (!HasVertex(u)) {return std::vector<std::vector<double>>(); }
   return _from_cost[u];
 };
 
 // Get number of vertices
 size_t SparseGraph::NumVertex() {
   return _to.size();
 };
 
 // Get number of arcs (directed edges)
 size_t SparseGraph::NumArc() {
   return _n_arc;
 };
 
 // Get number of edges (undirected, so arcs/2)
 size_t SparseGraph::NumEdge() {
   if (_n_arc % 2 != 0) {
     std::cout << "[ERROR] SparseGraph::NumEdge is not an integer but a fraction" << std::endl;
     throw std::runtime_error("[ERROR] SparseGraph::NumEdge is not an integer but a fraction");
   }
   return size_t(_n_arc / 2);
 };
 
 // Get cost dimension (number of cost components)
 size_t SparseGraph::CostDim() {
   return _cdim ;
 };
 
 // Get all vertex IDs
 std::vector<long> SparseGraph::AllVertex() 
 {
   std::vector<long> out;
   for (int i = 0; i < _to.size(); i++){
     out.push_back(i);
   }
   return out;
 };
 
 // Add a vertex to the graph
 void SparseGraph::AddVertex(long v) {
   if (!HasVertex(v)) {
     _to.resize(v+1);
     _to_cost.resize(v+1);
     _from.resize(v+1);
     _from_cost.resize(v+1);
   }
   return;
 };
 
 // Add an undirected edge between u and v with cost c
 void SparseGraph::AddEdge(long u, long v, std::vector<double> c) {
   AddArc(u,v,c);
   AddArc(v,u,c);
   return ;
 };
 
 // Add a directed arc from u to v with cost c
 void SparseGraph::AddArc(long u, long v, std::vector<double> c) {
   if (!HasVertex(u)) {
     AddVertex(u);
   }
   if (!HasVertex(v)) {
     AddVertex(v);
   }
 
   // Set cost dimension if not set
   if (_cdim == 0) {
     _cdim = c.size();
   }else{
     if (_cdim != c.size()) {
       std::cout << "[ERROR] SparseGraph::AddArc cdim does not match: " << _cdim << " != " << c.size() << std::endl;
       throw std::runtime_error("[ERROR] SparseGraph::AddArc cdim does not match");
     }
   }
 
   // Update outgoing edges for u
   bool updated = false;
   for (size_t idx = 0; idx < _to[u].size(); idx++){
     if (_to[u][idx] == v) {
       _to_cost[u][idx] = c;
       updated = true;
       break;
     }
   }
   if (!updated) {
     _to[u].push_back(v);
     _to_cost[u].push_back(c);
     _n_arc++;
   }
 
   // Update incoming edges for v
   updated = false;
   for (size_t idx = 0; idx < _from[v].size(); idx++){
     if (_from[v][idx] == u) {
       _from_cost[v][idx] = c;
       updated = true;
       break;
     }
   }
   if (!updated) {
     _from[v].push_back(u);
     _from_cost[v].push_back(c);
     _n_arc++;
   }
   return ;
 };
 
 // Create graph from undirected edges
 void SparseGraph::CreateFromEdges(std::vector<long> sources, 
     std::vector<long> targets, std::vector<std::vector<double>> costs)
 {
   _to.clear();
   _to_cost.clear();
   _from.clear();
   _from_cost.clear();
   // Find maximum vertex ID
   long max_id = 0;
   for (size_t i = 0; i < sources.size(); i++){
     if (sources[i] > max_id) { max_id = sources[i]; }
     if (targets[i] > max_id) { max_id = targets[i]; }
   }
   // Resize data structures
   _to.resize(max_id+1);
   _to_cost.resize(max_id+1);
   _from.resize(max_id+1);
   _from_cost.resize(max_id+1);
   // Add edges in both directions
   for (size_t i = 0; i < sources.size(); i++) {
     _to[sources[i]].push_back(targets[i]);
     _to_cost[sources[i]].push_back(costs[i]);
     _from[targets[i]].push_back(sources[i]);
     _from_cost[targets[i]].push_back(costs[i]);
     _to[targets[i]].push_back(sources[i]);
     _to_cost[targets[i]].push_back(costs[i]);
     _from[sources[i]].push_back(targets[i]);
     _from_cost[sources[i]].push_back(costs[i]);
   }
   return ;
 };
 
 // Create graph from directed arcs
 void SparseGraph::CreateFromArcs(std::vector<long> sources, 
     std::vector<long> targets, std::vector<std::vector<double>> costs)
 {
   _to.clear();
   _to_cost.clear();
   _from.clear();
   _from_cost.clear();
   // Find maximum vertex ID
   long max_id = 0;
   for (size_t i = 0; i < sources.size(); i++){
     if (sources[i] > max_id) { max_id = sources[i]; }
     if (targets[i] > max_id) { max_id = targets[i]; }
   }
   // Resize data structures
   _to.resize(max_id+1);
   _to_cost.resize(max_id+1);
   _from.resize(max_id+1);
   _from_cost.resize(max_id+1);
   // Add arcs
   for (size_t i = 0; i < sources.size(); i++) {
     _to[sources[i]].push_back(targets[i]);
     _to_cost[sources[i]].push_back(costs[i]);
     _from[targets[i]].push_back(sources[i]);
     _from_cost[targets[i]].push_back(costs[i]);
   }
   return ;
 };
 
 // Change cost dimension for all arcs
 void SparseGraph::ChangeCostDim(size_t new_cdim, double default_value) {
   for (size_t i = 0; i < _to.size(); i++) {
     for (size_t j = 0; j < _to[i].size(); j++) {
       _to_cost[i][j].resize(new_cdim, default_value);
     }
   }
   for (size_t i = 0; i < _from.size(); i++) {
     for (size_t j = 0; j < _from[i].size(); j++) {
       _from_cost[i][j].resize(new_cdim, default_value);
     }
   }
   _cdim = new_cdim;
   return ;
 };
 
 // Set cost for a specific arc
 bool SparseGraph::SetArcCost(long u, long v, const std::vector<double>& new_cost) {
   // Validate input
   if (new_cost.size() != _cdim){
     std::cout << "[ERROR] SparseGraph::SetArcCost new edge cost dimension does not match! Maybe call ChangeCostDim() at first." << std::endl;
     throw std::runtime_error("[ERROR] SparseGraph::SetArcCost new edge cost dimension does not match! Maybe call ChangeCostDim() at first.") ;
     return false;    
   }
   if ( (!HasVertex(u)) || (!HasVertex(v)) ) {
     std::cout << "[ERROR] SparseGraph::SetArcCost vertex does not exist!" << std::endl;
     throw std::runtime_error("[ERROR] SparseGraph::SetArcCost vertex does not exist!") ;
     return false;
   }
   if ( new_cost.size() != _cdim ) {
     std::cout << "[ERROR] SparseGraph::SetArcCost input vector size mismatch : " << new_cost.size() << " vs " << _cdim << std::endl;
     throw std::runtime_error("[ERROR] SparseGraph::SetArcCost input vector size mismatch!") ;
     return false;
   }
 
   // Update outgoing cost
   bool found = false;
   for (int i = 0; i < _to[u].size(); i++){
     if (_to[u][i] == v) {
       _to_cost[u][i] = new_cost;
       found = true;
       break;
     }
   }
   if (!found) {
     std::cout << "[ERROR] SparseGraph::SetArcCost arc does not exist!" << std::endl;
     throw std::runtime_error("[ERROR] SparseGraph::SetArcCost arc does not exist!") ;
   }
 
   // Update incoming cost
   found = false;
   for (int i = 0; i < _from[v].size(); i++){
     if (_from[v][i] == u) {
       _from_cost[v][i] = new_cost;
       found = true;
       break;
     }
   }
   if (!found) {
     std::cout << "[ERROR] SparseGraph::SetArcCost arc does not exist!" << std::endl;
     throw std::runtime_error("[ERROR] SparseGraph::SetArcCost arc does not exist!") ;
   }
   return true;
 };
 
 // Convert graph to string representation
 std::string SparseGraph::ToStr() const {
   std::string out;
   out += "=== SparseGraph Begin ===\n |V| = " + std::to_string(_to.size()) + " outgoing edges \n";
   // Print outgoing edges
   for (long v = 0; v < _to.size(); v++) {
     out += " -- " + std::to_string(v) + ":[";
     for (size_t idy = 0; idy < _to[v].size(); idy++){
       out += std::to_string(_to[v][idy]) + "(" + ToString(_to_cost[v][idy]) + "),";
     }
     out += "]\n";
   }
   // Print incoming edges
   out += " incoming edges \n";
   for (long v = 0; v < _to.size(); v++) {
     out += " -- " + std::to_string(v) + ":[";
     for (size_t idy = 0; idy < _from[v].size(); idy++){
       out += std::to_string(_from[v][idy]) + "(" + ToString(_from_cost[v][idy]) + "),";
     }
     out += "]\n";
   }
   out += "=== SparseGraph End ===";
   return out;
 };
 
 // Output operator overload
 std::ostream& operator<<(std::ostream& os, const SparseGraph& c) {
   os << c.ToStr();
   return os;
 };
 
 // ############################################################
 // ### Grid2d Implementation #################################
 // ############################################################
 
 // Constructor - defaults to 4-connected grid
 Grid2d::Grid2d() {
   SetKNeighbor(4); // by default
 };
 
 // Destructor
 Grid2d::~Grid2d() {};
 
 // Check if vertex exists in grid
 bool Grid2d::HasVertex(long v)
 {
   if ( (_k2r(v) < _occu_grid_ptr->size()) && (_k2c(v) < _occu_grid_ptr->at(0).size()) ){
     return true;
   }
   return false;
 };
 
 // Check if there's an arc between two grid cells
 bool Grid2d::HasArc(long v, long u) {
   // Get row/col for both vertices
   long r1 = _k2r(v);
   long c1 = _k2c(v);
   long r2 = _k2r(u);
   long c2 = _k2c(u);
   // Check all possible neighbor actions
   for (int i = 0; i < _act_r.size(); i++){
     bool b1 = (r2 == (r1+_act_r[i]));
     bool b2 = (c2 == (c1+_act_c[i]));
     if (b1 && b2) {return true;}
   }
   return false;
 };
 
 // Get successor cells (neighbors) in grid
 std::vector<long> Grid2d::GetSuccs(long v)
 {
   std::vector<long> out;
   long r = _k2r(v);
   long c = _k2c(v);
   // Check all neighbor directions
   for (size_t idx = 0; idx < _kngh; idx++) {
     long nr = r+_act_r[idx];
     if (nr >= _occu_grid_ptr->size()) {continue;} // out of bounds
     long nc = c+_act_c[idx];
     if (nc >= _occu_grid_ptr->at(0).size()) {continue;} // out of bounds
     if (_occu_grid_ptr->at(nr).at(nc) > 0) {continue;} // obstacle
     out.push_back(_rc2k(nr,nc)); // add valid neighbor
   }
   return out;
 };
 
 // Get predecessor cells (same as successors in undirected grid)
 std::vector<long> Grid2d::GetPreds(long v)
 {
   return GetSuccs(v);
 };
   
 // Get cost between two grid cells
 CostVec Grid2d::GetCost(long u, long v)
 {
   // v is the target vertex of arc (u,v)
   std::vector<double> out;
   long r = _k2r(v);
   long c = _k2c(v);
   long r2 = _k2r(v);
   long c2 = _k2c(v);
   // Diagonal move costs more (sqrt(2) ~ 1.4)
   if ((r != r2) && (c != c2)) {
     out.push_back(1.4*_cost_scale);
     return out;
   }else{
     out.push_back(1.0*_cost_scale); // straight move
     return out;
   }
 };
 
 // Get all successor costs for a grid cell
 std::vector< CostVec > Grid2d::GetSuccCosts(long u)
 {
   std::vector<std::vector<double>> out;
   long r = _k2r(u);
   long c = _k2c(u);
   for (size_t idx = 0; idx < _kngh; idx++) {
     long nr = r+_act_r[idx];
     long nc = c+_act_c[idx];
     if (! IsWithinBorder(nr, nc)) {continue;}; // skip if out of bounds
     if (_occu_grid_ptr->at(nr).at(nc) > 0) {continue;} // skip obstacles
     std::vector<double> c;
     if (idx <= 3) { 
       c.push_back(1.0*_cost_scale); // straight move
     }
     else if (idx <= 7) {
       c.push_back(1.4*_cost_scale); // diagonal move
     }
     else { throw std::runtime_error( "[ERROR], Grid2d _kngh > 8, not supported!" ); }
     out.push_back(c);
   }
   return out;
 };
 
 // Get all predecessor costs (same as successors in undirected grid)
 std::vector< CostVec > Grid2d::GetPredCosts(long u) {
   return GetSuccCosts(u);
 };
 
 // Get total number of cells in grid
 size_t Grid2d::NumVertex() {
   if (_occu_grid_ptr->size() == 0) {return 0;}
   return _occu_grid_ptr->size() * _occu_grid_ptr->at(0).size() ;
 } ;
 
 // Not implemented for grid
 size_t Grid2d::NumArc() {
   std::cout << "[ERROR], Grid2d::NumArc not implemented, TODO." << std::endl;
   throw std::runtime_error( "[ERROR], Grid2d::NumArc not implemented, TODO." );
   return 0;
 };
 
 // Not implemented for grid
 size_t Grid2d::NumEdge() {
   std::cout << "[ERROR], Grid2d::NumArc not implemented, TODO." << std::endl;
   throw std::runtime_error( "[ERROR], Grid2d::NumArc not implemented, TODO." );
   return 0;
 }
 
 // Grid always has 1D cost (distance)
 size_t Grid2d::CostDim() {
   return 1;
 };
 
 // Not implemented for grid
 std::vector<long> Grid2d::AllVertex() 
 {
   std::cout << "[ERROR], Grid2d::AllVertex not implemented, TODO." << std::endl;
   throw std::runtime_error( "[ERROR], Grid2d::AllVertex not implemented, TODO." );
   std::vector<long> out;
   return out;
 };
 
 //////////// Grid-specific methods ////////////
 
 // Set pointer to occupancy grid
 void Grid2d::SetOccuGridPtr(std::vector< std::vector<double> >* in)
 {
   _occu_grid_ptr = in;
   return ;
 };
 
 // Get pointer to occupancy grid
 std::vector< std::vector<double> >* Grid2d::GetOccuGridPtr()
 {
   return _occu_grid_ptr;
 };
 
 // Set occupancy grid from object (makes local copy)
 void Grid2d::SetOccuGridObject(std::vector< std::vector<double> >& in)
 {
   _mat_from_py = in; // make a local copy. To avoid pybind issue.
   SetOccuGridPtr(&_mat_from_py);
 };
 
 // Check if cell is within grid bounds
 bool Grid2d::IsWithinBorder(long nr, long nc) {
     if (nr >= _occu_grid_ptr->size() || nr < 0) {return false;}
     if (nc >= _occu_grid_ptr->at(0).size() || nc < 0) {return false;}
     return true;
 };
 
 // Set grid connectivity (4 or 8 neighbors)
 bool Grid2d::SetKNeighbor(int kngh) {
   if (kngh == 4 || kngh == 8) {
     _kngh = kngh;
     if (_kngh == 4) {
       _act_r = std::vector<long>({0,0,-1,1}); // left, right, up, down
       _act_c = std::vector<long>({-1,1,0,0});
     }else if (_kngh == 8) {
       // Add diagonal movements
       _act_r = std::vector<long>({ 0, 0,-1, 1, -1,-1, 1, 1});
       _act_c = std::vector<long>({-1, 1, 0, 0, -1, 1,-1, 1});
     }
     return true;
   }
   return false;
 };
 
 // Set cost scaling factor
 void Grid2d::SetCostScaleFactor(const double in) {
   _cost_scale = in;
 };
 
 // Convert row/col to vertex ID
 long Grid2d::_rc2k(const long r, const long c) const 
 {
   return r * _occu_grid_ptr->at(0).size() + c;
 };
 
 // Convert vertex ID to row
 long Grid2d::_k2r(const long k) const
 {
   return long(k / _occu_grid_ptr->at(0).size());
 };
 
 // Convert vertex ID to column
 long Grid2d::_k2c(const long k) const 
 {
   return k % _occu_grid_ptr->at(0).size();
 };
 
 /////////////////////////////////////////////////////////////////
 
 // ############################################################
 // ### HybridGraph2d Implementation ###########################
 // ############################################################
 
 // Constructor
 HybridGraph2d::HybridGraph2d() {};
 
 // Destructor
 HybridGraph2d::~HybridGraph2d() {};
 
 // Check if vertex exists in hybrid graph
 bool HybridGraph2d::HasVertex(long v) {
   if (v < 0) {return false;}
   if (v < _nid_ends.back()) {return true;}
   return false;
 };
 
 // Check if arc exists between vertices
 bool HybridGraph2d::HasArc(long v, long u) {
   if (!HasVertex(v)) {return false;}
   if (!HasVertex(u)) {return false;}
   // Check if in same subgraph
   if (_find_subgraph(v) == _find_subgraph(u)) {return true;}
   // Check inter-graph arcs
   for (int i = 0; i < _ig_arc_srcs.size(); i++){
     if (v == _ig_arc_srcs[i] && u == _ig_arc_tgts[i]){return true;}
   }
   return false;
 };
 
 // Get successors of vertex
 std::vector<long> HybridGraph2d::GetSuccs(long v) {
   std::vector<long> out;
   int idx = _find_subgraph(v);
   if (idx < 0) {return out;}
   long nid = _g2l_nid(v);
   int kk = _index_map[idx];
   // Get successors from appropriate subgraph
   if (kk >= 0){
     out = _grids[kk]->GetSuccs(nid);
   }else{
     out = _roadmaps[-kk-1]->GetSuccs(nid);
   }
   // Convert local IDs to global IDs
   for (int j = 0; j < out.size(); j++){
     out[j] += _nid_starts[idx];
   }
   // Add inter-graph arcs
   for (int j = 0; j < _ig_arc_srcs.size(); j++){
     if (_ig_arc_srcs[j] == v){
       out.push_back(_ig_arc_tgts[j]);
     }
   }
   return out;
 };
 
 // Get predecessors of vertex
 std::vector<long> HybridGraph2d::GetPreds(long v) {
   std::vector<long> out;
   int idx = _find_subgraph(v);
   if (idx < 0) {return out;}
   long nid = _g2l_nid(v);
   int kk = _index_map[idx];
   // Get predecessors from appropriate subgraph
   if (kk >= 0){
     out = _grids[kk]->GetPreds(nid);
   }else{
     out = _roadmaps[-kk-1]->GetPreds(nid);
   }
   // Convert local IDs to global IDs
   for (int j = 0; j < out.size(); j++){
     out[j] += _nid_starts[idx];
   }
   // Add inter-graph arcs
   for (int j = 0; j < _ig_arc_tgts.size(); j++){
     if (_ig_arc_tgts[j] == v){
       out.push_back(_ig_arc_srcs[j]);
     }
   }
   return out;
 };
 
 // Get cost between two vertices
 CostVec HybridGraph2d::GetCost(long u, long v) {
   std::vector<double> out;
   int idx = _find_subgraph(u);
   int idy = _find_subgraph(v);
   if (idx < 0 || idy < 0) {return out;}
   // If in same subgraph
   if (idx == idy){
     long uu = _g2l_nid(u);
     long vv = _g2l_nid(v);
     int kk = _index_map[idx];
     if (kk >= 0){
       return _grids[kk]->GetCost(uu,vv);
     }else{
       return _roadmaps[-kk-1]->GetCost(uu,vv);
     }
   }
   // Check inter-graph arcs
   for (int j = 0; j < _ig_arc_srcs.size(); j++){
     if (_ig_arc_srcs[j] == u && _ig_arc_tgts[j] == v){
       return _ig_costs[j];
     }
   }
   return out;
 };
 
 // Get all successor costs for vertex
 std::vector< CostVec > HybridGraph2d::GetSuccCosts(long u) {
   std::vector< CostVec > out;
   int idx = _find_subgraph(u);
   if (idx < 0) {return out;}
   long nid = _g2l_nid(u);
   int kk = _index_map[idx];
   // Get costs from appropriate subgraph
   if (kk >= 0){
     out = _grids[kk]->GetSuccCosts(nid);
   }else{
     out = _roadmaps[-kk-1]->GetSuccCosts(nid);
   }
   // Add inter-graph arc costs
   for (int j = 0; j < _ig_arc_srcs.size(); j++){
     if (_ig_arc_srcs[j] == u){
       out.push_back(_ig_costs[j]);
     }
   }
   return out;
 };
 
 // Get all predecessor costs for vertex
 std::vector< CostVec > HybridGraph2d::GetPredCosts(long u) {
   std::vector< CostVec > out;
   int idx = _find_subgraph(u);
   if (idx < 0) {return out;}
   long nid = _g2l_nid(u);
   int kk = _index_map[idx];
   // Get costs from appropriate subgraph
   if (kk >= 0){
     out = _grids[kk]->GetPredCosts(nid);
   }else{
     out = _roadmaps[-kk-1]->GetPredCosts(nid);
   }
   // Add inter-graph arc costs
   for (int j = 0; j < _ig_arc_tgts.size(); j++){
     if (_ig_arc_tgts[j] == u){
       out.push_back(_ig_costs[j]);
     }
   }
   return out;
 };
 
 // Get total number of vertices
 size_t HybridGraph2d::NumVertex() {
   if (_nid_ends.size() == 0) {
     return 0;
   }
   return _nid_ends.back();
 };
 
 // Get total number of arcs
 size_t HybridGraph2d::NumArc() {
   size_t out = 0;
   // Sum arcs from all grids
   for (auto p : _grids){
     out += p->NumArc();
   }
   // Sum arcs from all roadmaps
   for (auto p : _roadmaps){
     out += p->NumArc();
   }
   // Add inter-graph arcs
   out += _ig_arc_srcs.size();
   return out;
 };
 
 // Get total number of edges (undirected)
 size_t HybridGraph2d::NumEdge() {
   size_t n_arcs = NumArc();
   if (n_arcs % 2 != 0) {
     std::cout << "[ERROR] HybridGraph2d::NumEdge is not an integer but a fraction" << std::endl;
     throw std::runtime_error("[ERROR] HybridGraph2d::NumEdge is not an integer but a fraction");
   }
   return size_t(n_arcs / 2);
 };
 
 // Get cost dimension
 size_t HybridGraph2d::CostDim() {
   // assume all sub-grids and sub-roadmaps have the same cost dim.
   for (auto p : _grids){
     return p->CostDim();
   }
   for (auto p : _roadmaps){
     return p->CostDim();
   }
   for (auto c : _ig_costs){
     return c.size();
   }
   return 0;
 };
 
 // Not implemented
 std::vector<long> HybridGraph2d::AllVertex() {
   std::cout << "[ERROR], HybridGraph2d::AllVertex not implemented, TODO." << std::endl;
   throw std::runtime_error( "[ERROR], HybridGraph2d::AllVertex not implemented, TODO." );
   std::vector<long> out;
   return out;
 };
 
 // Add a grid to the hybrid graph
 void HybridGraph2d::AddGrid2d(Grid2d* g) {
   _grids.push_back(g);
   if (_nid_starts.size() == 0) {
     // First subgraph
     _nid_starts.push_back(0);
     _nid_ends.push_back(g->NumVertex());
     _index_map.push_back(0);
     return ;
   }
   // Subsequent subgraphs
   _nid_starts.push_back(_nid_ends.back());
   _nid_ends.push_back(_nid_starts.back()+g->NumVertex());
   _index_map.push_back(_grids.size()-1);
   return ;
 };
 
 // Add a roadmap to the hybrid graph
 void HybridGraph2d::AddSparseGraph(SparseGraph* g) {
   _roadmaps.push_back(g);
   if (_nid_starts.size() == 0) {
     // First subgraph
     _nid_starts.push_back(0);
     _nid_ends.push_back(g->NumVertex());
     _index_map.push_back(-1); // negative for roadmaps
     return ;
   } 
   // Subsequent subgraphs
   _nid_starts.push_back(_nid_ends.back());
   _nid_ends.push_back(_nid_starts.back()+g->NumVertex());
   _index_map.push_back(-_roadmaps.size()); // negative index for roadmaps
   return ;
 };
 
 // Add edge between different subgraphs
 void HybridGraph2d::AddExtraEdge(long u, long v, CostVec c) {
   _ig_arc_srcs.push_back(u);
   _ig_arc_tgts.push_back(v);
   _ig_costs.push_back(c);
   return ;
 };
 
 // Find which subgraph a vertex belongs to
 int HybridGraph2d::_find_subgraph(long v) {
   for (int i = 0; i < _nid_ends.size(); i++){
     if (v >= _nid_starts[i] && v < _nid_ends[i]){
       return i;
     }
   }
   return -1;
 };
 
 // Convert global vertex ID to local subgraph ID
 long HybridGraph2d::_g2l_nid(long v) {
   for (int i = 0; i < _nid_ends.size(); i++){
     if (v >= _nid_starts[i] && v < _nid_ends[i]){
       return v - _nid_starts[i];
     }
   }
   return -1;
 };
 
 /////////////////////////////////////////////////////////
 
 // ############################################################
 // ### DenseGraph Implementation ##############################
 // ############################################################
 
 // Constructor
 DenseGraph::DenseGraph() {};
 
 // Destructor
 DenseGraph::~DenseGraph() {};
 
 // Check if vertex exists
 bool DenseGraph::HasVertex(long v) {
   if (use_id_map) {return !(_id2ind.find(v) == _id2ind.end());}
   else {return (v >= 0) && (v < _cost_mat.size()); }
 } ;
 
 // Check if arc exists
 bool DenseGraph::HasArc(long v, long u) {
   if (! (HasVertex(v) && HasVertex(u))){
     return false;
   }
   size_t i,j;
   if (use_id_map){
     i = _id2ind[u];
     j = _id2ind[v];
   }else{
     i = u;
     j = v;
   }
   if (_cost_mat[j][i].size() == 0){
     return false;
   }
   return true;
 } ;
 
 // Get successors of vertex
 std::vector<long> DenseGraph::GetSuccs(long v) {
   std::vector<long> out;
   if (!HasVertex(v)) {return out;}
   size_t j;
   if (use_id_map){
     j = _id2ind[v];
   }else{
     j = v;
   }
   // Check all possible destinations
   for (size_t i = 0; i < _cost_mat.size(); i++){
     long u;
     if (use_id_map){
       u = _ind2id[i];
     }else{
       u = i;
     }
     if (u == v){continue;} // skip self
     if (_cost_mat[j][i].size() == 0){continue;} // skip if no arc
     out.push_back(u);
   }
   return out;
 } ;
 
 // Get predecessors of vertex
 std::vector<long> DenseGraph::GetPreds(long v) {
   std::vector<long> out;
   if (!HasVertex(v)) {return out;}
   size_t j;
   if (use_id_map){
     j = _id2ind[v];
   }else{
     j = v;
   }
   // Check all possible sources
   for (size_t i = 0; i < _cost_mat.size(); i++){
     long u = i;
     if (use_id_map){
       u = _ind2id[i];
     }
     if (u == v){continue;} // skip self
     if (_cost_mat[i][j].size() == 0){continue;} // skip if no arc
     out.push_back(u);
   }
   return out;
 };
 
 // Get cost between two vertices
 CostVec DenseGraph::GetCost(long u, long v) {
   size_t i,j;
   if (use_id_map){
     i = _id2ind[u];
     j = _id2ind[v];
   }else{
     i = u;
     j = v;
   }
   return _cost_mat[i][j];
 } ;
 
 // Get all successor costs
 std::vector< CostVec > DenseGraph::GetSuccCosts(long u) {
   std::vector< CostVec > out;
   size_t i;
   if (use_id_map){
     i = _id2ind[u];
   }else{
     i = u;
   }
   // Get all outgoing arc costs
   for (size_t j = 0; j < _cost_mat[i].size(); j++){
     long v = j;
     if (use_id_map){
       v = _ind2id[j];
     }
     if (u == v){continue;} // skip self
     if (_cost_mat[i][j].size() == 0){continue;} // skip if no arc
     out.push_back(_cost_mat[i][j]);
   }
   return out;
 } ;
 
 // Get all predecessor costs
 std::vector< CostVec > DenseGraph::GetPredCosts(long u) {
   std::vector< CostVec > out;
   size_t i;
   if (use_id_map){
     i = _id2ind[u];
   }{
     i = u;
   }
   // Get all incoming arc costs
   for (size_t j = 0; j < _cost_mat.size(); j++){
     long v = j;
     if (use_id_map){
       v = _ind2id[j];
     }
     if (u == v){continue;} // skip self
     if (_cost_mat[j][i].size() == 0){continue;} // skip if no arc
     out.push_back(_cost_mat[j][i]);
   }
   return out;
 } ;
 
 // Get number of vertices
 size_t DenseGraph::NumVertex() {
   return _cost_mat.size();
 } ;
 
 // Get number of arcs (n^2 - n for complete graph)
 size_t DenseGraph::NumArc() {
   return _cost_mat.size()*_cost_mat.size() - _cost_mat.size();
 } ;
 
 // Get number of edges (undirected)
 size_t DenseGraph::NumEdge() {
   return NumArc()/2;
 } ;
 
 // Get cost dimension
 size_t DenseGraph::CostDim() {
   for (size_t i = 0; i < _cost_mat.size(); i++){
     for (size_t j = 0; j < _cost_mat.size(); j++){
       if (_cost_mat[i][j].size() != 0) {
         return _cost_mat[i][j].size();
       }
     }
   }
   return 0;
 } ;
 
 // Get all vertex IDs
 std::vector<long> DenseGraph::AllVertex() {
   std::vector<long> out;
   for (size_t i = 0; i < _cost_mat.size(); i++){
     if (use_id_map){
       out.push_back(_ind2id[i]);
     }else{
       out.push_back(i);
     }
   }
   return out;
 } ;
 
 // Create from undirected edges
 void DenseGraph::CreateFromEdges(std::vector<long> sources, 
   std::vector<long> targets, std::vector< std::vector<double> > costs)
 {
   _id2ind.clear();
   _ind2id.clear();
   size_t n = 0;
   // Build vertex ID mapping
   for (int ii = 0; ii < sources.size(); ii++){
     long v = sources[ii];
     if (use_id_map){
       if (_id2ind.find(v) != _id2ind.end()){
         continue;
       }
     }else{
       if (v < n){
         continue;
       }else{
         n = v+1;
       }
     }
     if (use_id_map){
       _id2ind[sources[ii]] = n;
       _ind2id[n] = sources[ii];
       n++;
     }
   }
   // Same for targets
   for (int ii = 0; ii < targets.size(); ii++){
     long v = targets[ii];
     if (use_id_map){
       if (_id2ind.find(v) != _id2ind.end()){
         continue;
       }
     }else{
       if (v < n){
         continue;
       }else{
         n = v+1;
       }
     }
     if (use_id_map){
       _id2ind[targets[ii]] = n;
       _ind2id[n] = targets[ii];
       n++;
     }
   }
   // Initialize cost matrix
   _cost_mat.resize(n);
   for (int ii = 0; ii < n ; ii++){
     _cost_mat[ii].resize(n);
   }
   // Add edges in both directions
   for (int ii = 0; ii < sources.size(); ii++){
     size_t i1,i2;
     if (use_id_map){
       i1 = _id2ind[sources[ii]];
       i2 = _id2ind[targets[ii]];
     }else{
       i1 = sources[ii];
       i2 = targets[ii];
     }
     _cost_mat[i1][i2] = costs[ii];
     _cost_mat[i2][i1] = costs[ii];
   }
   return ;
 };
 
 // Create from directed arcs
 void DenseGraph::CreateFromArcs(std::vector<long> sources, 
   std::vector<long> targets, std::vector< std::vector<double> > costs)
 {
   _id2ind.clear();
   _ind2id.clear();
   size_t n = 0;
   // Build vertex ID mapping
   for (int ii = 0; ii < sources.size(); ii++){
     long v = sources[ii];
     if (use_id_map){
       if (_id2ind.find(v) != _id2ind.end()){
         continue;
       }
     }else{
       if (v < n){
         continue;
       }else{
         n = v+1;
       }
     }
     if (use_id_map){
       _id2ind[sources[ii]] = n;
       _ind2id[n] = sources[ii];
       n++;
     }
   }
   // Same for targets
   for (int ii = 0; ii < targets.size(); ii++){
     long v = targets[ii];
     if (use_id_map){
       if (_id2ind.find(v) != _id2ind.end()){
         continue;
       }
     }else{
       if (v < n){
         continue;
       }else{
         n = v+1;
       }
     }
     if (use_id_map){
       _id2ind[targets[ii]] = n;
       _ind2id[n] = targets[ii];
       n++;
     }
   }
   // Initialize cost matrix
   _cost_mat.resize(n);
   for (int ii = 0; ii < n ; ii++){
     _cost_mat[ii].resize(n);
   }
   // Add arcs (one direction)
   for (int ii = 0; ii < sources.size(); ii++){
     size_t i1,i2;
     if (use_id_map){
       i1 = _id2ind[sources[ii]];
       i2 = _id2ind[targets[ii]];
     }else{
       i1 = sources[ii];
       i2 = targets[ii];
     }
     _cost_mat[i1][i2] = costs[ii];
   }
   return ;
 };
 
 // Convert graph to string representation
 std::string DenseGraph::ToStr() const {
   std::string out;
   out += "=== DenseGraph Begin === |V| = " + std::to_string(_cost_mat.size()) + " \n";
   out += " All Vertices: ";
   // List all vertices
   for (size_t i = 0; i < _cost_mat.size(); i++){
     if (use_id_map){
       out += std::to_string(i) + "(" + std::to_string(_ind2id.at(i)) + ") ";
     }else{
       out += std::to_string(i) + "(" + std::to_string(i) + ") ";      
     }
   }
   out += "\n";
   out += " Cost Mat: \n";
   // Print cost matrix
   for (size_t i = 0; i < _cost_mat.size(); i++){
     for (size_t j = 0; j < _cost_mat[i].size(); j++){
       out += ToString(_cost_mat[i][j]) + " ";
     }
     out += "\n";
   }
   out += "=== DenseGraph End ===";
   return out;
 };
 
 // Output operator overload
 std::ostream& operator<<(std::ostream& os, const DenseGraph& c) {
   os << c.ToStr();
   return os;
 };
 
 } // end namespace zr