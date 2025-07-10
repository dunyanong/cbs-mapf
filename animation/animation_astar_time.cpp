#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "movingai_map_parser.hpp"
#include "movingai_scen_parser.hpp"
#include "search_astar.hpp"

using namespace std;
using namespace cv;
using namespace movingai;
using namespace raplab;

const int CELL_SIZE = 10;
const Scalar OBSTACLE_COLOR(0, 0, 0); // Black
const Scalar EMPTY_COLOR(255, 255, 255); // White
const Scalar PATH_COLOR(0, 255, 0); // Green
const Scalar AGENT_COLOR(0, 0, 255); // Red

void drawGrid(const gridmap& map, Mat& img) {
    for (int y = 0; y < map.height_; y++) {
        for (int x = 0; x < map.width_; x++) {
            Rect cell(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
            if (map.is_obstacle({x, y})) {
                rectangle(img, cell, OBSTACLE_COLOR, FILLED);
            } else {
                rectangle(img, cell, EMPTY_COLOR, FILLED);
            }
        }
    }
}

void drawPath(const vector<State>& path, Mat& img) {
    for (const auto& state : path) {
        Rect cell(state.x * CELL_SIZE, state.y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
        rectangle(img, cell, PATH_COLOR, FILLED);
    }
}

void drawAgent(const State& agent, Mat& img) {
    Rect cell(agent.x * CELL_SIZE, agent.y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
    rectangle(img, cell, AGENT_COLOR, FILLED);
}

vector<State> convertPath(const vector<long>& path, int width) {
    vector<State> result;
    for (long id : path) {
        result.push_back({static_cast<vid>(id % width), static_cast<vid>(id / width)});

    }
    return result;
}

int main() {
    string mapFile = "/Users/ongdunyan/Downloads/LocalCodes/cbs-mapf/data/arena/arena.map";
    string scenFile = "/Users/ongdunyan/Downloads/LocalCodes/cbs-mapf/data/arena/arena.map.scen";

    // Load map
    gridmap map(mapFile);

    // Load scenario
    scenario_manager scenMgr;
    scenMgr.load_scenario(scenFile);

    // Create OpenCV window
    Mat img(map.height_ * CELL_SIZE, map.width_ * CELL_SIZE, CV_8UC3);
    namedWindow("Pathfinding Animation", WINDOW_AUTOSIZE);

    // Iterate through test cases
    for (int i = 0; i < scenMgr.num_experiments(); i++) {
        auto expr = scenMgr.get_experiment(i);

        // Initialize A* planner
        Grid2d g;
        vector<vector<double>> occupancyGrid(map.height_, vector<double>(map.width_, 0));
        for (int y = 0; y < map.height_; y++) {
            for (int x = 0; x < map.width_; x++) {
                occupancyGrid[y][x] = map.is_obstacle({x, y}) ? 1 : 0;
            }
        }
        g.SetOccuGridPtr(&occupancyGrid);

        AstarGrid2d planner;
        planner.SetGraphPtr(&g);

        // Perform pathfinding
        int startId = expr->starty() * map.width_ + expr->startx();
        int goalId = expr->goaly() * map.width_ + expr->goalx();
        auto path = planner.PathFinding(startId, goalId);

        // Convert path to grid coordinates
        auto gridPath = convertPath(path, map.width_);

        // Draw initial grid
        drawGrid(map, img);
        drawPath(gridPath, img);

        // Animate agent movement
        for (const auto& state : gridPath) {
            Mat frame = img.clone();
            drawAgent(state, frame);
            imshow("Pathfinding Animation", frame);
            waitKey(100); // Delay for animation
        }

        waitKey(500); // Pause before next test case
    }

    destroyAllWindows();
    return 0;
}