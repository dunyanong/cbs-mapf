#ifndef CBS_H
#define CBS_H

#include "search_astar_st.hpp"
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>

namespace raplab {

class CBSSolver {
public:
    CBSSolver();
    ~CBSSolver();

    void SetGraphPtr(StateSpaceST* graph);
    void SetTimeLimit(double time_limit);
    
    // Add agents with their start and goal positions
    void AddAgent(long start_id, long goal_id);
    
    // Solve the MAPF problem
    bool Solve();
    
    // Get the path for a specific agent
    std::vector<long> GetPath(int agent_id);

private:
    struct Constraint {
        int agent_id;
        long vertex;
        long time;
    };

    struct CBSNode {
        double cost;
        std::vector<std::vector<long>> solution;
        std::vector<Constraint> constraints;
        std::shared_ptr<CBSNode> parent;
        int conflict_agent1;
        int conflict_agent2;
        long conflict_vertex;
        long conflict_time;
    };

    struct CBSNodeCompare {
        bool operator()(const std::shared_ptr<CBSNode>& a, const std::shared_ptr<CBSNode>& b) {
            return a->cost > b->cost;
        }
    };

    StateSpaceST* graph_;
    double time_limit_;
    std::vector<std::pair<long, long>> agents_; // start and goal positions
    std::vector<std::vector<long>> paths_;
    std::priority_queue<std::shared_ptr<CBSNode>, 
                        std::vector<std::shared_ptr<CBSNode>>, 
                        CBSNodeCompare> open_list_;
    
    bool FindConflict(const std::vector<std::vector<long>>& solution, 
                     int& agent1, int& agent2, 
                     long& vertex, long& time);
    std::vector<long> FindPath(int agent_id, const std::vector<Constraint>& constraints);
};

} // namespace raplab

#endif // CBS_H