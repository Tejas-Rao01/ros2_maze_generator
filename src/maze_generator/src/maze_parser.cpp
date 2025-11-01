#include "maze_generator/maze_parser.hpp"
#include <iostream>

MazeParser::MazeParser(const std::vector<std::vector<Cell>>& grid,
                       double cellSize,
                       double wallThickness,
                       double wallHeight)
    : mazeGrid(grid),
      m_cellSize(cellSize),
      m_wallThickness(wallThickness),
      m_wallHeight(wallHeight)
{
    m_height = mazeGrid.size();
    m_width = mazeGrid[0].size();
}

std::vector<Wall> MazeParser::extractWalls() {
    auto horizontal_walls = extract_horizontal_walls();
    auto vertical_walls = extract_vertical_walls(); 
    std::vector<Wall> walls(horizontal_walls);
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
            bool is_wall = mazeGrid[row][col].walls[South];
            if (!is_wall || col == m_width-1){
                int wall_len = col - start_col; 
                if (wall_len > 0)
                    walls.push_back({start_col, row+1, col, row+1});
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
            bool is_wall = mazeGrid[row][col].walls[East];
            if (!is_wall || row == m_height-1){
                int wall_len = row - start_row; 
                if (wall_len > 0)
                    walls.push_back({col+1, start_row, col+1, row});
                start_row = row+1; 
            }
        }
    }
    return walls; 
}

std::vector<Wall> MazeParser::generate_border_walls() {
    std::vector<Wall> walls;
    // Top border
    walls.push_back({1, 0, static_cast<int>(m_width), 0});
    // Bottom border
    walls.push_back({0, static_cast<int>(m_height), static_cast<int>(m_width)-1, static_cast<int>(m_height)});
    // Left border
    walls.push_back({0, 0, 0, static_cast<int>(m_height)});
    // Right border
    walls.push_back({static_cast<int>(m_width), 0, static_cast<int>(m_width), static_cast<int>(m_height)});
    return walls;
}

// int main() {
//     // Make a simple 4x4 grid
//     int rows = 4, cols = 4;
//     std::vector<std::vector<Cell>> grid(rows, std::vector<Cell>(cols));

//     // Knock down some walls to make paths
//     grid[0][0].walls[East] = false;   // open between (0,0) and (0,1)
//     grid[0][1].walls[West] = false;

//     grid[0][1].walls[South] = false;  // open between (0,1) and (1,1)
//     grid[1][1].walls[North] = false;

//     grid[1][1].walls[East] = false;   // open between (1,1) and (1,2)
//     grid[1][2].walls[West] = false;

//     grid[1][2].walls[South] = false;
//     grid[2][2].walls[North] = false;

//     // Parse walls
//     MazeParser parser(grid);
//     auto walls = parser.extractWalls();

//     // Print walls
//     std::cout << "Extracted walls:\n";
//     for (const auto& w : walls) {
//         std::cout << "(" << w.start_x << "," << w.start_y << ") -> ("
//                   << w.end_x << "," << w.end_y << ")\n";
//     }

//     // Simple ASCII visualization
//     std::cout << "\nMaze visualization:\n";
//     for (int r = 0; r < rows; r++) {
//         // Print north walls
//         for (int c = 0; c < cols; c++) {
//             std::cout << "+";
//             if (grid[r][c].walls[North]) std::cout << "---";
//             else std::cout << "   ";
//         }
//         std::cout << "+\n";

//         // Print west walls and spaces
//         for (int c = 0; c < cols; c++) {
//             if (grid[r][c].walls[West]) std::cout << "|";
//             else std::cout << " ";
//             std::cout << "   ";
//         }
//         // Last east wall
//         std::cout << "|\n";
//     }
//     // Bottom border
//     for (int c = 0; c < cols; c++) std::cout << "+---";
//     std::cout << "+\n";

//     return 0;
// }
