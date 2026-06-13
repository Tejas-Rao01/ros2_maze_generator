#ifndef MAZE_GENERATOR_HPP
#define MAZE_GENERATOR_HPP

#include <vector>
#include <array>
#include <utility>
#include "maze_generator/utils.hpp"

class MazeGenerator {
public:
    MazeGenerator(int width, int height);

    std::vector<Cell> generate();

private:
    bool isValid(int row, int col) const;
    int getIndex(int row, int col) const;

    int m_width, m_height;
    std::vector<Cell> m_grid;
};

#endif // MAZE_GENERATOR_HPP
