#include <opencv2/opencv.hpp>
#include <vector>
#include "movingai_map_parser.hpp"

namespace opencv_animation {
    const int CELL_SIZE = 10;
    const cv::Scalar OBSTACLE_COLOR(0, 0, 0);
    const cv::Scalar EMPTY_COLOR(255, 255, 255);
    const cv::Scalar PATH_COLOR(0, 255, 0);
    const cv::Scalar AGENT_COLOR(0, 0, 255);
    const cv::Scalar START_COLOR(255, 0, 255);
    const cv::Scalar GOAL_COLOR(0, 255, 255);

    void drawGrid(const movingai::gridmap& map, cv::Mat& img);
    void drawPath(const std::vector<movingai::State>& path, cv::Mat& img, const movingai::State& start, const movingai::State& goal);
    void drawAgent(const movingai::State& agent, cv::Mat& img);
    std::vector<movingai::State> convertPath(const std::vector<long>& path, int width);
    void drawStartAndGoal(const movingai::State& start, const movingai::State& goal, cv::Mat& img);
}
