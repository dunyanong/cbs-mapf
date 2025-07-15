#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "movingai_map_parser.hpp"
#include "movingai_scen_parser.hpp"
#include "search_astar_st.hpp"
#include <cstdlib>
#include "animation_opencv.hpp"

using namespace std;
using namespace cv;
using namespace movingai;
using namespace raplab;
using namespace opencv_animation;

const char* mapFile = std::getenv("MAPFILE");
const char* scenFile = std::getenv("SCENFILE");

int main() {
    if (!mapFile || !scenFile) {
        cerr << "Error: MAPFILE or SCENFILE environment variable is not set!" << endl;
        return -1;
    }

    // Load map
    gridmap grid_map(mapFile);

    // Load scenario
    scenario_manager scenMgr;
    scenMgr.load_scenario(scenFile);

    // Create OpenCV window
    Mat img(grid_map.height_ * CELL_SIZE, grid_map.width_ * CELL_SIZE, CV_8UC3);
    namedWindow("Pathfinding Animation", WINDOW_AUTOSIZE);

    // Get the test case
    auto expr = scenMgr.get_experiment(159); // 159 is max experiment index

    if (!expr) {
        cerr << "Error: expr is null!" << endl;
        return -1;
    }

    // Log start and goal positions
    cout << "Start Position: (" << expr->startx() << ", " << expr->starty() << ")" << endl;
    cout << "Goal Position: (" << expr->goalx() << ", " << expr->goaly() << ")" << endl;

    // Check if start or goal is an obstacle
    if (grid_map.is_obstacle({static_cast<vid>(expr->startx()), static_cast<vid>(expr->starty())})) {
        cerr << "Error: Start position is an obstacle!" << endl;
        return -1;
    }
    if (grid_map.is_obstacle({static_cast<vid>(expr->goalx()), static_cast<vid>(expr->goaly())})) {
        cerr << "Error: Goal position is an obstacle!" << endl;
        return -1;
    }

    // Initialize A* planner
    StateSpaceST g;
    vector<vector<double>> occupancyGrid(grid_map.height_, vector<double>(grid_map.width_, 0));

    g.SetOccuGridPtr(&occupancyGrid);

    auto pp = AstarSTGrid2d();
    pp.SetGraphPtr(&g);
    cout << "Graph initialized with occupancy grid." << endl;
    pp.AddNodeCstr(3,3);
    pp.AddNodeCstr(12,3);
    pp.AddEdgeCstr(24,25,6);
    pp.AddNodeCstr(99,20);
    pp.SetHeuWeight(1.2);

    // Perform pathfinding
    int startId = expr->starty() * grid_map.width_ + expr->startx();
    int goalId = expr->goaly() * grid_map.width_ + expr->goalx();
    cout << "Start ID: " << startId << ", Goal ID: " << goalId << endl;
    auto path = pp.PathFinding(startId, goalId);

    // Convert path to grid coordinates
    auto gridPath = convertPath(path, grid_map.width_);
    if (path.empty()) {
        cerr << "Pathfinding failed: No path found!" << endl;
        return -1;
    }    

    // Draw initial grid
    State startState = {
        static_cast<vid>(expr->startx()),
        static_cast<vid>(expr->starty())
    };
    State goalState = {
        static_cast<vid>(expr->goalx()),
        static_cast<vid>(expr->goaly())
    };
    drawGrid(grid_map, img);
    drawPath(gridPath, img, startState, goalState);
    drawStartAndGoal(startState, goalState, img);

    // Display the image to ensure markers are visible
    imshow("Pathfinding Animation", img);
    waitKey(100);

    if (gridPath.empty()) {
        cerr << "No path found between start and goal!" << endl;
        return -1;
    }

    // Animate agent movement
    for (const auto& state : gridPath) {
        Mat frame = img.clone();
        drawAgent(state, frame);
        imshow("Pathfinding Animation", frame);
        waitKey(500); // Delay for animation
    }

    waitKey(0); 
    destroyAllWindows();
    return 0;    
    
}
