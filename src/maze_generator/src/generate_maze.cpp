#include "maze_generator/maze_generator.hpp"
#include "maze_generator/maze_parser.hpp"
#include "maze_generator/maze_writer.hpp"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

int main(int argc, char **argv) {
    // Default parameters
    int width = 5;
    int height = 5;
    double board_res = 1.0;
    double map_res = 1.0;
    double wall_thickness = 0.2;
    std::string maze_prefix = "maze";

    if (argc > 1) maze_prefix = argv[1];
    if (argc > 2) width = std::stoi(argv[2]);
    if (argc > 3) height = std::stoi(argv[3]);
    if (argc > 4) board_res = std::stod(argv[4]);
    if (argc > 5) map_res = std::stod(argv[5]);
    if (argc > 6) wall_thickness = std::stod(argv[6]);

    std::cout << "\n=== Maze Generator ===\n";
    std::cout << "Output prefix:     " << maze_prefix << "\n";
    std::cout << "Maze width:        " << width << "\n";
    std::cout << "Maze height:       " << height << "\n";
    std::cout << "Board resolution:  " << board_res << "\n";
    std::cout << "Map resolution:    " << map_res << "\n";
    std::cout << "Wall thickness:    " << wall_thickness << "\n";
    std::cout << "======================\n";

    // --- Step 1: Generate and parse maze ---
    MazeGenerator generator(width, height);
    auto grid = generator.generate();
    MazeParser parser(grid, width, height, board_res, wall_thickness);
    auto walls = parser.extractWalls();

    // --- Step 2: Write metadata ---
    MetaData meta;
    meta.board_width      = width;
    meta.board_height     = height;
    meta.board_res        = board_res;
    meta.map_resolution   = map_res;
    meta.num_walls        = static_cast<int>(walls.size());
    meta.wall_thickness   = wall_thickness;
    meta.files_path       = maze_prefix;
    meta.walls            = walls;

    MazeWriter writer;
    writer.writeFile(meta);

    std::cout << "✅ Maze generated and written to: " << maze_prefix << std::endl;
    return 0;
}
