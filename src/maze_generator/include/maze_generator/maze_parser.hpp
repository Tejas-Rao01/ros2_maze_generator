#ifndef MAZE_PARSER_HPP
#define MAZE_PARSER_HPP

#include <vector>
#include "maze_generator/utils.hpp"

class MazeParser {
public:
    MazeParser(const std::vector<Cell>& grid,
               int width, int height,
               double cellSize = 1.0,
               double wallThickness = 0.2,
               double wallHeight = 1.0);

    std::vector<Wall> extractWalls();

private:
    std::vector<Wall> extract_horizontal_walls();
    std::vector<Wall> extract_vertical_walls();
    std::vector<Wall> generate_border_walls();
    int getIndex(int row, int col) const;

    std::vector<Cell> m_grid;
    int m_width, m_height;
    double m_cellSize, m_wallThickness, m_wallHeight;
};

#endif // MAZE_PARSER_HPP
