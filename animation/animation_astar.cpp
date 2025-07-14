#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include "movingai_map_parser.hpp"
#include "movingai_scen_parser.hpp"
#include "search_astar.hpp"
#include <cstdlib>

using namespace std;
using namespace cv;
using namespace movingai;
using namespace raplab;

const int CELL_SIZE = 10;
const Scalar OBSTACLE_COLOR(0, 0, 0);
const Scalar EMPTY_COLOR(255, 255, 255);
const Scalar PATH_COLOR(0, 255, 0);
const Scalar AGENT_COLOR(0, 0, 255);
const Scalar START_COLOR(255, 0, 255);
const Scalar GOAL_COLOR(0, 255, 255);

const char* mapFile = std::getenv("MAPFILE");
const char* scenFile = std::getenv("SCENFILE");

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

void drawPath(const vector<State>& path, Mat& img, const State& start, const State& goal) {
    for (const auto& state : path) {
        if ((state.x == start.x && state.y == start.y) ||
            (state.x == goal.x && state.y == goal.y)) {
            continue;
        }
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

void drawStartAndGoal(const State& start, const State& goal, Mat& img) {
    Rect startCell(start.x * CELL_SIZE, start.y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
    Rect goalCell(goal.x * CELL_SIZE, goal.y * CELL_SIZE, CELL_SIZE, CELL_SIZE);

    rectangle(img, startCell, START_COLOR, FILLED);
    rectangle(img, startCell, Scalar(0, 0, 0), 1);

    rectangle(img, goalCell, GOAL_COLOR, FILLED);
    rectangle(img, goalCell, Scalar(0, 0, 0), 1);
}

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