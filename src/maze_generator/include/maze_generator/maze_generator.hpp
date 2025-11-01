#pragma once
#include <vector>
#include <array>
#include <utility> // for std::pair
#include "maze_generator/utils.hpp"

class MazeGenerator {
public:
    MazeGenerator(int width, int height);

    std::vector<std::vector<Cell>> generate();

private:
    void generateDFS(int row, int col);
    bool isValid(int row, int col);

    int m_width, m_height;
    std::vector<std::vector<Cell>> m_grid;
    std::vector<std::vector<bool>> m_visited;
};
