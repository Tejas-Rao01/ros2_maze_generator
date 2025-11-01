#ifndef UTILS_HPP
#define UTILS_HPP

#include <vector>
#include <array>

// Directions enum
enum Direction { North = 0, South = 1, East = 2, West = 3 };

// Offsets for movement (row, col)
const std::array<std::pair<int,int>,4> OFFSETS = {{
    {-1, 0}, // North
    { 1, 0}, // South
    { 0, 1}, // East
    { 0,-1}  // West
}};

// Opposite directions (used when knocking down walls)
const std::array<Direction,4> OPPOSITE = {{
    South, North, West, East
}};

class Cell {
public:
    Cell() {
        walls.fill(true); // all walls present initially
    }
    std::array<bool,4> walls; // N, S, E, W
};

struct Wall {
    int start_x, start_y, end_x, end_y; 
};




#endif // UTILS_HPP