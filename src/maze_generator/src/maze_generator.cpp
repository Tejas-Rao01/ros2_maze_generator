#include "maze_generator/maze_generator.hpp"
#include <random>
#include <algorithm>
#include <stack>

MazeGenerator::MazeGenerator(int width, int height)
    : m_width(width), m_height(height),
      m_grid(width * height)
{
    // Open entrance and exit
    m_grid[getIndex(0, 0)].walls[North] = false;
    m_grid[getIndex(height - 1, width - 1)].walls[South] = false;
}

std::vector<Cell> MazeGenerator::generate() {
    std::vector<bool> visited(m_width * m_height, false);
    std::stack<std::pair<int, int>> stack;
    
    std::random_device rd;
    std::mt19937 gen(rd());

    int startRow = 0;
    int startCol = 0;
    stack.push({startRow, startCol});
    visited[getIndex(startRow, startCol)] = true;

    while (!stack.empty()) {
        auto [row, col] = stack.top();
        
        std::vector<int> neighbors;
        for (int d = 0; d < 4; ++d) {
            int nRow = row + OFFSETS[d].first;
            int nCol = col + OFFSETS[d].second;
            if (isValid(nRow, nCol) && !visited[getIndex(nRow, nCol)]) {
                neighbors.push_back(d);
            }
        }

        if (!neighbors.empty()) {
            std::shuffle(neighbors.begin(), neighbors.end(), gen);
            int dir = neighbors[0];
            int nRow = row + OFFSETS[dir].first;
            int nCol = col + OFFSETS[dir].second;

            // Knock down walls
            m_grid[getIndex(row, col)].walls[dir] = false;
            m_grid[getIndex(nRow, nCol)].walls[OPPOSITE[dir]] = false;

            visited[getIndex(nRow, nCol)] = true;
            stack.push({nRow, nCol});
        } else {
            stack.pop();
        }
    }

    return m_grid;
}

bool MazeGenerator::isValid(int row, int col) const {
    return row >= 0 && row < m_height && col >= 0 && col < m_width;
}

int MazeGenerator::getIndex(int row, int col) const {
    return row * m_width + col;
}
