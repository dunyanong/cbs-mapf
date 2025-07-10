#include "search_astar_st.hpp"
#include "debug.hpp"
#include <opencv2/opencv.hpp>
#include <sstream>
#include <iomanip>

int TestAstarSTGrid2d();
void SaveFrame(int timestep,
    const std::vector<std::vector<double>>& occu_grid,
    const std::vector<long>& path,
    int agent_index = -1);

int main(){
    system("mkdir -p frames");
    cv::Mat testimg(400, 400, CV_8UC3, cv::Scalar(255,255,255));
    cv::imwrite("frames/test.png", testimg);
    std::cout << "Saved test.png to frames folder." << std::endl;    

    TestAstarSTGrid2d();

    return 0;
};


int TestAstarSTGrid2d(){

    std::cout << "####### TestAstarSTGrid2d() Begin #######" << std::endl;
    raplab::SimpleTimer timer;
    timer.Start();

    // raplab::Grid2d
    raplab::StateSpaceST g;
    std::vector<std::vector<double> > occupancy_grid;
    occupancy_grid.resize(10);
    for (int i = 0; i < 10; i++){
        occupancy_grid[i].resize(10, 0);
    }
    for (int i = 0; i < 10; i++){
        if (i == 5){continue;}
        occupancy_grid[5][i] = 1;
    }
    g.SetOccuGridPtr(&occupancy_grid);

    timer.Start();
    auto pp = raplab::AstarSTGrid2d();
    pp.SetGraphPtr(&g);
    pp.AddNodeCstr(3,3);
    pp.AddNodeCstr(12,3);
    pp.AddEdgeCstr(24,25,6);
    pp.AddNodeCstr(99,20);
    pp.SetHeuWeight(1.2);
    auto p = pp.PathFinding(0,99,10); // Start from vertex 0 to vertex 99 with a time limit of 10 seconds

    system("mkdir -p frames");

    std::cout << "Path size: " << p.size() << std::endl;
    if (p.empty()) {
        std::cout << "Path is empty! No frames will be saved." << std::endl;
    }    

    for (int t = 0; t < p.size(); ++t) {
        SaveFrame(t, occupancy_grid, p);
        
    }   


    auto d_all = pp.GetDistAll();
    for (auto vv : p) {
    std::cout << " v = " << vv << " dist = " << d_all[vv] << std::endl;
    }
    for (size_t jj = 0; jj < d_all.size(); jj++) {
    std::cout << " d_all[" << jj << "] = " << d_all[jj] << std::endl;
    }

    std::cout << " p = " << p << std::endl;

    timer.PrintDuration();

    std::cout << "####### TestAstarSTGrid2d() End #######" << std::endl;

    return 1;
};

void SaveFrame(int timestep,
            const std::vector<std::vector<double>>& occu_grid,
            const std::vector<long>& path,
            int agent_index)
{
    int rows = occu_grid.size();
    int cols = occu_grid[0].size();
    int cell_size = 40;

    cv::Mat img(rows * cell_size, cols * cell_size, CV_8UC3, cv::Scalar(255,255,255));

    // Draw obstacles
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (occu_grid[r][c] == 1) {
                cv::rectangle(img, cv::Rect(c * cell_size, r * cell_size, cell_size, cell_size), cv::Scalar(0,0,0), cv::FILLED);
            }
        }
    }

    // Draw path so far
    for (int i = 0; i <= timestep && i < path.size(); ++i) {
        int vid = path[i];
        int r = vid / cols;
        int c = vid % cols;
        cv::circle(img, cv::Point(c * cell_size + cell_size / 2, r * cell_size + cell_size / 2), cell_size / 3,
                (i == timestep ? cv::Scalar(0, 0, 255) : cv::Scalar(200, 100, 100)), -1);
    }

    // Save frame
    std::ostringstream filename;
    filename << "frames/frame_" << std::setw(3) << std::setfill('0') << timestep << ".png";
    bool success = cv::imwrite(filename.str(), img);
    if (!success) {
        std::cerr << "Failed to save image: " << filename.str() << std::endl;
    }    
}
