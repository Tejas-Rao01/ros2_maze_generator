#include "maze_generator/maze_parser.hpp"
#include <iostream>

MazeParser::MazeParser(const std::vector<Cell>& grid,
                       int width, int height,
                       double cellSize,
                       double wallThickness,
                       double wallHeight)
    : m_grid(grid),
      m_width(width),
      m_height(height),
      m_cellSize(cellSize),
      m_wallThickness(wallThickness),
      m_wallHeight(wallHeight)
{
}

std::vector<Wall> MazeParser::extractWalls() {
    auto walls = extract_horizontal_walls();
    auto vertical_walls = extract_vertical_walls(); 
    walls.insert(walls.end(), vertical_walls.begin(), vertical_walls.end());

    auto border_walls = generate_border_walls();
    walls.insert(walls.end(), border_walls.begin(), border_walls.end());

    return walls;
}

std::vector<Wall> MazeParser::extract_horizontal_walls() {
    std::vector<Wall> walls; 
    for (int row = 0; row < m_height - 1; row++){
        int start_col = 0; 
        for(int col = 0; col < m_width; col++){
            bool is_wall = m_grid[getIndex(row, col)].walls[South];
            if (!is_wall || col == m_width-1){
                int wall_len = col - start_col; 
                if (is_wall && col == m_width - 1) wall_len++;
                if (wall_len > 0)
                    walls.push_back({start_col, row+1, start_col + wall_len, row+1});
                start_col = col+1; 
            }
        }
    }
    return walls; 
}

std::vector<Wall> MazeParser::extract_vertical_walls() {
    std::vector<Wall> walls; 
    for (int col = 0; col < m_width - 1; col++){
        int start_row = 0; 
        for(int row = 0; row < m_height; row++){
            bool is_wall = m_grid[getIndex(row, col)].walls[East];
            if (!is_wall || row == m_height-1){
                int wall_len = row - start_row; 
                if (is_wall && row == m_height - 1) wall_len++;
                if (wall_len > 0)
                    walls.push_back({col+1, start_row, col+1, start_row + wall_len});
                start_row = row+1; 
            }
        }
    }
    return walls; 
}

std::vector<Wall> MazeParser::generate_border_walls() {
    std::vector<Wall> walls;
    // Top border (skip entrance at 0,0)
    walls.push_back({1, 0, m_width, 0});
    // Bottom border (skip exit at height-1, width-1)
    walls.push_back({0, m_height, m_width-1, m_height});
    // Left border
    walls.push_back({0, 0, 0, m_height});
    // Right border
    walls.push_back({m_width, 0, m_width, m_height});
    return walls;
}

int MazeParser::getIndex(int row, int col) const {
    return row * m_width + col;
}
