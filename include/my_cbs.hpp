#include "search_astar_st.hpp"
#include <vector>
#include <unordered_map>
#include <queue>

namespace raplab {
    struct Constraint{
        int agent;
        long vertex;
        long time;
    };

    struct CBSNode {
        std::vector<Constraint> constraints; // Constraints for this node
        std::vector<std::vector<long>> paths; // Paths for all agents
        double cost;

        // by default gives you a max-heap, we want a min-heap
        bool operator<(const CBSNode& other) const {
            return cost > other.cost; // For priority queue, we want the lowest cost first
        };

    };
    class CBS {
        public:
            CBS(PlannerGraph* input_graph, int num_agents);
            std::vector<std::vector<long>> Solve(
                const std::vector<long>& starts,
                const std::vector<long>& goals
            );

        private:
            PlannerGraph* input_graph;
            int num_agents;

            std::vector<long>LowLevelSearch(
                int agent,
                const std::vector<Constraint>& constraints,
                long start,
                long goal
            );
            bool DetectConflict(
                const std::vector<std::vector<long>>& paths,
                int& agent1,
                int& agent2,
                long& conflict_vertex,
                long& conflict_time
            );
    };
}