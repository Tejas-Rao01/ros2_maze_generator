#pragma once 

#include <fstream>
#include <vector>
#include <memory> 
#include <string>
#include <filesystem> 
#include <sstream>

// bunch of includes

class MazeToMap{
public:
    MazeToMap(); 
    ~MazeToMap(); 
    std::unique_ptr<std::vector<int>> get_map(const std::string& maze_file_path);
    int get_grid_height_cells();
    int get_grid_width_cells();
    double get_resolution();


private: 
    
    std::unique_ptr<std::vector<int>> generate(const std::vector<std::vector<int>> &walls);
    // std::string find_value(const std::string& key);
    int m_board_width_pixels;
    int m_board_height_pixels; 
    double m_board_resolution; // pixel to meters 
    double m_resolution; 

    int m_num_walls; 
    double m_wall_thickness_meters;
};

