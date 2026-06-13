#ifndef MAZE_TO_MAP_HPP
#define MAZE_TO_MAP_HPP

#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <sstream>
#include <cstdint>

class MazeToMap {
public:
    MazeToMap();
    ~MazeToMap();

    std::vector<int8_t> get_map(const std::string& maze_file_path);

    int get_grid_height_cells() const;
    int get_grid_width_cells() const;
    double get_resolution() const;

private:
    std::vector<int8_t> generate(const std::vector<std::vector<int>>& walls);

    int m_grid_width_cells;
    int m_grid_height_cells;
    double m_board_resolution;
    double m_resolution;
    double m_wall_thickness_meters;
};

#endif // MAZE_TO_MAP_HPP
