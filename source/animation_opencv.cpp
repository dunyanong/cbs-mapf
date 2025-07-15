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

namespace opencv_animation {

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
}