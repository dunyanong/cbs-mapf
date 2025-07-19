#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "movingai_map_parser.hpp"
#include "movingai_scen_parser.hpp"
#include "search_astar_st.hpp"
#include "cbs.hpp"
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

    // Get two test cases for multiple agents
    auto expr1 = scenMgr.get_experiment(0); // First agent
    auto expr2 = scenMgr.get_experiment(1); // Second agent

    if (!expr1 || !expr2) {
        cerr << "Error: Couldn't load experiments!" << endl;
        return -1;
    }

    // Initialize CBS solver
    StateSpaceST g;
    vector<vector<double>> occupancyGrid(grid_map.height_, vector<double>(grid_map.width_, 0));
    g.SetOccuGridPtr(&occupancyGrid);

    CBSSolver cbs;
    cbs.SetGraphPtr(&g);
    cbs.SetTimeLimit(10.0);

    // Add agents
    int startId1 = expr1->starty() * grid_map.width_ + expr1->startx();
    int goalId1 = expr1->goaly() * grid_map.width_ + expr1->goalx();
    cbs.AddAgent(startId1, goalId1);

    int startId2 = expr2->starty() * grid_map.width_ + expr2->startx();
    int goalId2 = expr2->goaly() * grid_map.width_ + expr2->goalx();
    cbs.AddAgent(startId2, goalId2);

    cout << "Solving for Agent 1: Start=" << startId1 << ", Goal=" << goalId1 << endl;
    cout << "Solving for Agent 2: Start=" << startId2 << ", Goal=" << goalId2 << endl;

    // Solve the MAPF problem
    if (!cbs.Solve()) {
        cerr << "Failed to find solution!" << endl;
        return -1;
    }

    // Get paths for both agents
    auto path1 = cbs.GetPath(0);
    auto path2 = cbs.GetPath(1);

    // Convert paths to grid coordinates
    auto gridPath1 = convertPath(path1, grid_map.width_);
    auto gridPath2 = convertPath(path2, grid_map.width_);

    if (path1.empty() || path2.empty()) {
        cerr << "Pathfinding failed for one or both agents!" << endl;
        return -1;
    }

    // Draw initial grid
    State startState1 = {
        static_cast<vid>(expr1->startx()),
        static_cast<vid>(expr1->starty())
    };
    State goalState1 = {
        static_cast<vid>(expr1->goalx()),
        static_cast<vid>(expr1->goaly())
    };
    State startState2 = {
        static_cast<vid>(expr2->startx()),
        static_cast<vid>(expr2->starty())
    };
    State goalState2 = {
        static_cast<vid>(expr2->goalx()),
        static_cast<vid>(expr2->goaly())
    };

    drawGrid(grid_map, img);
    drawPath(gridPath1, img, startState1, goalState1, Scalar(255, 0, 0)); // Blue for agent 1
    drawPath(gridPath2, img, startState2, goalState2, Scalar(0, 0, 255)); // Red for agent 2
    drawStartAndGoal(startState1, goalState1, img, Scalar(255, 0, 0));
    drawStartAndGoal(startState2, goalState2, img, Scalar(0, 0, 255));

    // Display the image
    imshow("Pathfinding Animation", img);
    waitKey(100);

    // Animate both agents
    size_t max_steps = max(gridPath1.size(), gridPath2.size());
    for (size_t t = 0; t < max_steps; t++) {
        Mat frame = img.clone();
        
        // Draw agent 1 if still moving
        if (t < gridPath1.size()) {
            drawAgent(gridPath1[t], frame, Scalar(255, 0, 0));
        }
        
        // Draw agent 2 if still moving
        if (t < gridPath2.size()) {
            drawAgent(gridPath2[t], frame, Scalar(0, 0, 255));
        }
        
        imshow("Pathfinding Animation", frame);
        waitKey(500); // Delay for animation
    }

    waitKey(0); 
    destroyAllWindows();
    return 0;    
}