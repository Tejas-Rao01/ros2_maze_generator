
#include "maze_generator/maze_to_map.hpp"
#include <iostream> 

void visualize_map(const std::vector<int>& map_data, int width, int height) {
    std::cout << "\n--- Generated Occupancy Grid (" << width << "x" << height << ") ---\n";
    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            int index = r * width + c;
            if (index < map_data.size()) {
                // '1' is occupied, '0' is free
                char symbol = (map_data[index] == 1) ? '#' : ' '; 
                std::cout << symbol;
            }
        }
        std::cout << "\n";
    }
    std::cout << "---------------------------------------------------------\n";
}

int main() {

    const std::string MAZE_PREFIX = "/home/tejas/apps/maze_generator_ws/src/maze_generator/mazes/maze"; 
    // 2. Instantiate and Parse
    MazeToMap maze_to_map;
    std::unique_ptr<std::vector<int>> map_data_ptr = nullptr;
    int map_width = 0;
    int map_height = 0;

    try {
        map_data_ptr = maze_to_map.get_map(MAZE_PREFIX);
        std::cout << "Size: " << map_data_ptr->size() << std::endl;
        
        map_width = maze_to_map.get_grid_width_cells();
        map_height = maze_to_map.get_grid_height_cells();

        if (map_data_ptr && !map_data_ptr->empty()) {
            visualize_map(*map_data_ptr, map_width, map_height);
        } else {
            std::cerr << "ERROR: Map data is empty or parsing failed.\n";
        }
        
    } catch (const std::exception& e) {
        std::cerr << "An exception occurred during map generation: " << e.what() << '\n';
        // Clean up the created file
        return 1;
    }

    return 0;
}