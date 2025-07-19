#include "cbs.hpp"
#include <iostream>
#include <algorithm>

namespace raplab {

CBSSolver::CBSSolver() : graph_(nullptr), time_limit_(10.0) {}

CBSSolver::~CBSSolver() {}

void CBSSolver::SetGraphPtr(StateSpaceST* graph) {
    graph_ = graph;
}

void CBSSolver::SetTimeLimit(double time_limit) {
    time_limit_ = time_limit;
}

void CBSSolver::AddAgent(long start_id, long goal_id) {
    agents_.emplace_back(start_id, goal_id);
}

bool CBSSolver::Solve() {
    // Initialize root node
    auto root = std::make_shared<CBSNode>();
    root->cost = 0;
    root->solution.resize(agents_.size());
    root->constraints.clear();
    
    // Find initial paths without constraints
    for (int i = 0; i < agents_.size(); ++i) {
        root->solution[i] = FindPath(i, root->constraints);
        if (root->solution[i].empty()) {
            std::cerr << "No path found for agent " << i << std::endl;
            return false;
        }
        root->cost += root->solution[i].size();
    }
    
    open_list_.push(root);
    
    while (!open_list_.empty()) {
        auto current = open_list_.top();
        open_list_.pop();
        
        // Check for conflicts
        int agent1, agent2;
        long vertex, time;
        if (!FindConflict(current->solution, agent1, agent2, vertex, time)) {
            // No conflicts found, solution is valid
            paths_ = current->solution;
            return true;
        }
        
        // Create new nodes for each agent in conflict
        for (int agent : {agent1, agent2}) {
            auto new_node = std::make_shared<CBSNode>();
            new_node->constraints = current->constraints;
            new_node->constraints.push_back({agent, vertex, time});
            new_node->solution = current->solution;
            
            // Replan path for the constrained agent
            new_node->solution[agent] = FindPath(agent, new_node->constraints);
            if (new_node->solution[agent].empty()) {
                continue; // Skip if no path exists with these constraints
            }
            
            // Calculate new cost
            new_node->cost = 0;
            for (int i = 0; i < agents_.size(); ++i) {
                new_node->cost += new_node->solution[i].size();
            }
            
            new_node->parent = current;
            new_node->conflict_agent1 = agent1;
            new_node->conflict_agent2 = agent2;
            new_node->conflict_vertex = vertex;
            new_node->conflict_time = time;
            
            open_list_.push(new_node);
        }
    }
    
    return false;
}

std::vector<long> CBSSolver::GetPath(int agent_id) {
    if (agent_id < 0 || agent_id >= paths_.size()) {
        return {};
    }
    return paths_[agent_id];
}

bool CBSSolver::FindConflict(const std::vector<std::vector<long>>& solution, 
                            int& agent1, int& agent2, 
                            long& vertex, long& time) {
    // Find the maximum path length
    size_t max_length = 0;
    for (const auto& path : solution) {
        if (path.size() > max_length) {
            max_length = path.size();
        }
    }
    
    // Check for conflicts at each time step
    for (size_t t = 0; t < max_length; ++t) {
        std::unordered_map<long, int> positions;
        
        for (int i = 0; i < solution.size(); ++i) {
            const auto& path = solution[i];
            if (t >= path.size()) {
                continue; // Agent has reached its goal
            }
            
            long pos = path[t];
            if (positions.count(pos)) {
                // Conflict found
                agent1 = positions[pos];
                agent2 = i;
                vertex = pos;
                time = t;
                return true;
            }
            positions[pos] = i;
        }
    }
    
    return false;
}

std::vector<long> CBSSolver::FindPath(int agent_id, const std::vector<Constraint>& constraints) {
    AstarSTGrid2d planner;
    planner.SetGraphPtr(graph_);
    
    // Add constraints for this agent
    for (const auto& c : constraints) {
        if (c.agent_id == agent_id) {
            planner.AddNodeCstr(c.vertex, c.time);
        }
    }
    
    // Find path
    return planner.PathFinding(agents_[agent_id].first, agents_[agent_id].second);
}

} // namespace raplab