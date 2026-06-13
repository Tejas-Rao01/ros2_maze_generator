#include "maze_generator/maze_to_map.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

MazeToMap::MazeToMap() : m_grid_width_cells(0), m_grid_height_cells(0), m_board_resolution(0.0), m_resolution(0.0), m_wall_thickness_meters(0.0) {}
MazeToMap::~MazeToMap() {}

std::vector<int8_t> MazeToMap::get_map(const std::string& maze_file_path) {
    std::string file_path = maze_file_path + "_metadata.json";
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open file " << file_path << std::endl;
        return {};
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    auto find_value = [&](const std::string& key) -> std::string {
        size_t start = content.find(key);
        if (start == std::string::npos) return "";
        start = content.find(":", start);
        if (start == std::string::npos) return "";
        size_t end = content.find_first_of(",}\n", start + 1);
        if (end == std::string::npos) end = content.size();
        std::string val = content.substr(start + 1, end - start - 1);
        val.erase(std::remove_if(val.begin(), val.end(), [](unsigned char c){ return std::isspace(c) || c == '"'; }), val.end());
        return val;
    };

    int board_width_pixels = std::stoi(find_value("\"board_width\""));
    int board_height_pixels = std::stoi(find_value("\"board_height\""));
    m_board_resolution = std::stod(find_value("\"board_res\""));
    m_resolution = std::stod(find_value("\"map_resolution\""));
    m_wall_thickness_meters = std::stod(find_value("\"wall_thickness\""));

    size_t walls_start = content.find("\"walls\"");
    if (walls_start == std::string::npos) return {};
    walls_start = content.find("[", walls_start);
    
    std::vector<std::vector<int>> walls;
    std::vector<int> current_wall;
    std::string temp;
    int bracket_depth = 0;
    
    for (size_t i = walls_start; i < content.size(); ++i) {
        char c = content[i];
        if (c == '[') {
            bracket_depth++;
        } else if (c == ']') {
            if (!temp.empty()) {
                current_wall.push_back(std::stoi(temp));
                temp.clear();
            }
            if (current_wall.size() == 4) {
                walls.push_back(current_wall);
            }
            current_wall.clear();
            bracket_depth--;
            if (bracket_depth == 0) break;
        } else if (std::isdigit(c) || c == '-' || c == '.') {
            temp += c;
        } else if (c == ',' && bracket_depth > 1) {
            if (!temp.empty()) {
                current_wall.push_back(std::stoi(temp));
                temp.clear();
            }
        }
    }

    double grid_width_meters = board_width_pixels * m_board_resolution;
    double grid_height_meters = board_height_pixels * m_board_resolution;

    int wall_half = std::max(1, static_cast<int>((m_wall_thickness_meters / m_resolution) / 2.0));
    m_grid_width_cells = static_cast<int>(std::ceil(grid_width_meters / m_resolution)) + wall_half * 2;
    m_grid_height_cells = static_cast<int>(std::ceil(grid_height_meters / m_resolution)) + wall_half * 2;

    return generate(walls);
}

std::vector<int8_t> MazeToMap::generate(const std::vector<std::vector<int>>& walls) {
    std::vector<int8_t> grid(m_grid_width_cells * m_grid_height_cells, 0);
    int wall_half = std::max(1, static_cast<int>((m_wall_thickness_meters / m_resolution) / 2.0));
    int wall_thickness_cells = wall_half * 2;

    auto draw_wall = [&](int r_start, int r_end, int c_start, int c_end) {
        for (int r = std::max(0, r_start); r < std::min(m_grid_height_cells, r_end); ++r) {
            for (int c = std::max(0, c_start); c < std::min(m_grid_width_cells, c_end); ++c) {
                grid[r * m_grid_width_cells + c] = 100;
            }
        }
    };

    for (const auto& wall : walls) {
        if (wall.size() < 4) continue;

        int start_c = static_cast<int>((wall[0] * m_board_resolution) / m_resolution);
        int start_r = static_cast<int>((wall[1] * m_board_resolution) / m_resolution);
        int end_c = static_cast<int>((wall[2] * m_board_resolution) / m_resolution);
        int end_r = static_cast<int>((wall[3] * m_board_resolution) / m_resolution);

        if (start_c == end_c) { // Vertical
            int r_min = std::min(start_r, end_r);
            int r_max = std::max(start_r, end_r);
            draw_wall(r_min, r_max + wall_thickness_cells, start_c, start_c + wall_thickness_cells);
        } else if (start_r == end_r) { // Horizontal
            int c_min = std::min(start_c, end_c);
            int c_max = std::max(start_c, end_c);
            draw_wall(start_r, start_r + wall_thickness_cells, c_min, c_max + wall_thickness_cells);
        }
    }

    // The original code had a 180-degree rotation. 
    // Let's implement it more efficiently if needed, but first let's just make it work.
    // ROS OccupancyGrid is usually row-major, starting from bottom-left.
    // Gazebo/SDF might have different conventions. 
    // I'll keep the rotation logic if it was intended to match the Gazebo world.
    
    std::vector<int8_t> rotated_grid(grid.size());
    for (int r = 0; r < m_grid_height_cells; ++r) {
        for (int c = 0; c < m_grid_width_cells; ++c) {
            int src_idx = r * m_grid_width_cells + c;
            int dst_idx = (m_grid_height_cells - 1 - r) * m_grid_width_cells + (m_grid_width_cells - 1 - c);
            rotated_grid[dst_idx] = grid[src_idx];
        }
    }

    return rotated_grid;
}

int MazeToMap::get_grid_height_cells() const { return m_grid_height_cells; }
int MazeToMap::get_grid_width_cells() const { return m_grid_width_cells; }
double MazeToMap::get_resolution() const { return m_resolution; }
