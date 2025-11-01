#pragma once
#include <vector>
#include "maze_generator/utils.hpp"

class MazeParser {
public:
    MazeParser(const std::vector<std::vector<Cell>>& grid,
               double cellSize = 1.0,
               double wallThickness = 0.2,
               double wallHeight = 1.0);

    std::vector<Wall> extractWalls();

private:
    std::vector<Wall> extract_horizontal_walls();
    std::vector<Wall> extract_vertical_walls();
    std::vector<Wall> generate_border_walls();

    std::vector<std::vector<Cell>> mazeGrid;
    int m_width, m_height;
    double m_cellSize, m_wallThickness, m_wallHeight;
};
