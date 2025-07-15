#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "movingai_map_parser.hpp"
#include "movingai_scen_parser.hpp"
#include "search_astar.hpp"
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
    auto expr = scenMgr.get_experiment(55); // For simplicity, using the 55th experiment

    if (!expr) {
        cerr << "Error: expr is null!" << endl;
        return -1;
    }

    // Initialize A* planner
    Grid2d g;
    vector<vector<double>> occupancyGrid(grid_map.height_, vector<double>(grid_map.width_, 0));
    for (int y = 0; y < grid_map.height_; y++) {
        for (int x = 0; x < grid_map.width_; x++) {
            occupancyGrid[y][x] = grid_map.is_obstacle({x, y}) ? 1 : 0;
        }
    }
    g.SetOccuGridPtr(&occupancyGrid);

    AstarGrid2d planner;
    planner.SetGraphPtr(&g);

    // Perform pathfinding
    int startId = expr->starty() * grid_map.width_ + expr->startx();
    int goalId = expr->goaly() * grid_map.width_ + expr->goalx();
    auto path = planner.PathFinding(startId, goalId);

    // Convert path to grid coordinates
    auto gridPath = convertPath(path, grid_map.width_);

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
        waitKey(100); // Delay for animation
    }

    waitKey(1000);
    destroyAllWindows();
    return 0;
}