#include "maze_generator/maze_generator.hpp"
#include <random>
#include <algorithm>

MazeGenerator::MazeGenerator(int width, int height)
    : m_width(width), m_height(height),
      m_grid(height, std::vector<Cell>(width)),
      m_visited(height, std::vector<bool>(width, false))
{
    // Open entrance and exit
    m_grid[0][0].walls[North] = false;
    m_grid[height-1][width-1].walls[South] = false;
}

std::vector<std::vector<Cell>> MazeGenerator::generate() {
    generateDFS(0, 0);
    return m_grid;
}

void MazeGenerator::generateDFS(int row, int col) {
    m_visited[row][col] = true;

    // Randomized directions
    std::array<int,4> dirs = {0,1,2,3};
    std::shuffle(dirs.begin(), dirs.end(), std::mt19937(std::random_device{}()));

    for (int d : dirs) {
        int newRow = row + OFFSETS[d].first;
        int newCol = col + OFFSETS[d].second;

        if (isValid(newRow, newCol) && !m_visited[newRow][newCol]) {
            // Knock down walls
            m_grid[row][col].walls[d] = false;
            m_grid[newRow][newCol].walls[OPPOSITE[d]] = false;

            generateDFS(newRow, newCol);
        }
    }
}

bool MazeGenerator::isValid(int row, int col) {
    return row >= 0 && row < m_height && col >= 0 && col < m_width;
}
