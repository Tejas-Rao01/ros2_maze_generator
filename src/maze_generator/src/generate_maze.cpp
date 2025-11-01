#include "maze_generator/maze_generator.hpp"
#include "maze_generator/maze_parser.hpp"
#include "maze_generator/maze_writer.hpp"
#include <iostream>

int main(int argc, char **argv) {
    // Default parameters
    int width = 5;
    int height = 5;
    double board_res = 1.0;
    double map_res = 1.0;
    double wall_thickness = 0.2;
    std::string maze_prefix = "/home/tejas/apps/maze_generator_ws/src/maze_generator/mazes/maze";

    // Argument order:
    // ./maze_file_generator <maze_prefix> <width> <height> <board_res> <map_res> <wall_thickness>'
    std::cout << argv[1] << std::endl;
    if (argc > 1) width = std::stoi(argv[1]);
    if (argc > 2) height = std::stoi(argv[2]);
    if (argc > 3) board_res = std::stod(argv[3]);
    if (argc > 3) map_res = std::stod(argv[4]);
    if (argc > 4) wall_thickness = std::stod(argv[5]);

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
    MazeParser parser(grid);
    auto walls = parser.extractWalls();

    // --- Step 2: Write metadata ---
    MetaData meta;
    meta.board_width      = width;
    meta.board_height     = height;
    meta.board_res        = board_res;
    meta.map_resolution   = map_res;
    meta.num_walls        = walls.size();
    meta.wall_thickness   = wall_thickness;
    meta.files_path       = maze_prefix;
    meta.walls            = walls;

    MazeWriter writer;
    writer.writeFile(meta);

    std::cout << "✅ Maze generated and written to: " << maze_prefix << std::endl;
    return 0;
}
