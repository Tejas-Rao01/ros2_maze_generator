#include "maze_generator/maze_to_map.hpp"
#include <iostream>
#include <algorithm>

MazeToMap::MazeToMap(){}

MazeToMap::~MazeToMap(){}


std::unique_ptr<std::vector<int>> MazeToMap::get_map(const std::string& maze_file_path) {
    std::string file_path = maze_file_path + "_metadata.json";
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open file " << file_path << std::endl;
        return nullptr;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    auto find_value = [&](const std::string& key) -> std::string {
        size_t start = content.find(key);
        if (start == std::string::npos) return "";
        start = content.find(":", start);
        size_t end = content.find_first_of(",}\n", start + 1);
        if (end == std::string::npos) end = content.size();
        std::string val = content.substr(start + 1, end - start - 1);
        // strip whitespace and quotes
        val.erase(remove_if(val.begin(), val.end(), ::isspace), val.end());
        val.erase(remove(val.begin(), val.end(), '"'), val.end());
        return val;
    };

    auto parse_walls = [](const std::string& json_str) -> std::vector<std::vector<int>> {
        std::vector<std::vector<int>> walls;
        std::vector<int> current;
        std::string temp;

        std::cout << "json string" << std::endl;
        std::cout << json_str << std::endl;

        for (char c : json_str) {
            if (std::isdigit(c) || c == '-') {
                temp += c;
            } else if (!temp.empty()) {
                current.push_back(std::stoi(temp));
                temp.clear();
                if (current.size() == 4) {
                    walls.push_back(current);
                    current.clear();
                }
            }
        }
        if (!temp.empty()) current.push_back(std::stoi(temp));
        if (current.size() == 4) walls.push_back(current);

        std::cout << "Parsed " << walls.size() << " walls:\n";
        for (const auto& w : walls)
            std::cout << "[" << w[0] << "," << w[1] << "] -> ["
                      << w[2] << "," << w[3] << "]\n";

        return walls;
    };

    // --- Scalar values ---
    m_board_width_pixels   = std::stoi(find_value("\"board_width\""));
    m_board_height_pixels  = std::stoi(find_value("\"board_height\""));
    m_board_resolution     = std::stod(find_value("\"board_res\""));
    m_resolution           = std::stod(find_value("\"map_resolution\""));
    m_num_walls            = std::stoi(find_value("\"num_walls\""));
    m_wall_thickness_meters= std::stod(find_value("\"wall_thickness\""));

    std::cout << "board_width: " << m_board_width_pixels << "\n";
    std::cout << "board_height: " << m_board_height_pixels << "\n";
    std::cout << "board_res: " << m_board_resolution << "\n";
    std::cout << "map_resolution: " << m_resolution << "\n";
    std::cout << "num_walls: " << m_num_walls << "\n";
    std::cout << "wall_thickness: " << m_wall_thickness_meters << "\n";

    // --- Extract the walls array ---
    size_t start = content.find("\"walls\"");
    if (start == std::string::npos) {
        std::cerr << "No walls found!\n";
        return nullptr;
    }

    // Find the opening '[' of the walls array
    start = content.find("[", start);
    if (start == std::string::npos) {
        std::cerr << "Malformed walls: missing '['\n";
        return nullptr;
    }

    // Find the matching closing ']'
    int bracket_depth = 0;
    size_t end = start;
    for (; end < content.size(); ++end) {
        if (content[end] == '[') {
            bracket_depth++;
        } else if (content[end] == ']') {
            bracket_depth--;
            if (bracket_depth == 0)
                break; // Found matching closing bracket
        }
    }

    if (bracket_depth != 0) {
        std::cerr << "Malformed walls section: unmatched brackets!\n";
        return nullptr;
    }

    std::string wall_section = content.substr(start, end - start + 1);
    auto walls = parse_walls(wall_section);

    std::cout << "number of walls: " << walls.size() << std::endl;
    std::cout << "start generating" << std::endl;

    auto map_ptr = generate(walls);
    return map_ptr;
}

std::unique_ptr<std::vector<int>> MazeToMap::generate(
    const std::vector<std::vector<int>> &walls)
{

    std::cout << "Generating occupancy grid..." << std::endl;
    
    int wall_half = std::max(1, static_cast<int>((m_wall_thickness_meters / m_resolution) / 2.0));
    
    double grid_width_meters  = m_board_width_pixels  * m_board_resolution;
    double grid_height_meters = m_board_height_pixels * m_board_resolution;

    int grid_width_cells  = static_cast<int>(grid_width_meters  / m_resolution + wall_half * 2);
    int grid_height_cells = static_cast<int>(grid_height_meters / m_resolution + wall_half * 2);

    m_board_width_pixels = grid_width_cells;
    m_board_height_pixels = grid_height_cells;

    // Half wall thickness in grid cells
    

    // --- 2. Initialize 2D grid ---
    auto grid = std::make_unique<std::vector<std::vector<int>>>(
        grid_height_cells, std::vector<int>(grid_width_cells, 0)
    );


    // --- 4. Lambdas for internal walls ---
    auto draw_horizontal_wall = [&](int row, int startx, int endx)
    {
        // Horizontal wall lies *above* row → between (row, row+1)
        for (int wallr = row; wallr < row + wall_half * 2; ++wallr) {
            if (wallr < 0 || wallr >= grid_height_cells) continue;
            for (int wallc = startx; wallc < endx + wall_half * 2; ++wallc) {
                if (wallc < 0 || wallc >= grid_width_cells) continue;
                (*grid)[wallr][wallc] = 1;
            }
        }
    };

    auto draw_vertical_wall = [&](int starty, int endy, int col)
    {
        // Vertical wall lies *to the right* of column → between (col, col+1)
        for (int wallc = col; wallc < col + wall_half * 2; ++wallc) {
            if (wallc < 0 || wallc >= grid_width_cells) continue;
            for (int wallr = starty; wallr < endy + wall_half * 2; ++wallr) {
                if (wallr < 0 || wallr >= grid_height_cells) continue;
                (*grid)[wallr][wallc] = 1;
            }
        }
    };


    std::cout << "Drawing internal walls..." << std::endl;
    // --- 5. Draw internal walls ---
    for (const auto &wall : walls)
    {

        if (wall.size() < 4) continue;

        int startx = (wall[0] * m_board_resolution) / m_resolution;
        int starty = (wall[1] * m_board_resolution) / m_resolution;
        int endx = (wall[2] * m_board_resolution) / m_resolution;
        int endy = (wall[3] * m_board_resolution) / m_resolution;

        if (startx == endx && starty == endy) {
            continue;
        } else if (startx == endx) {
            draw_vertical_wall(starty, endy, startx);
        } else if (starty == endy) {
            draw_horizontal_wall(starty, startx, endx);
        }
    }

    // Rotate the grid 180 degrees (flip both vertically and horizontally)
    for (int r = 0; r < grid_height_cells / 2; ++r) {
        std::swap((*grid)[r], (*grid)[grid_height_cells - 1 - r]);
    }
    for (int r = 0; r < grid_height_cells; ++r) {
        std::reverse((*grid)[r].begin(), (*grid)[r].end());
    }

    auto return_grid = std::make_unique<std::vector<int>>(grid_height_cells * grid_width_cells, 0);

    int idx = 0;
    for (int r = 0; r < grid_height_cells; ++r)
        for (int c = 0; c < grid_width_cells; ++c)
            (*return_grid)[idx++] = (*grid)[r][c];

    std::cout << (*return_grid).size() << std::endl;
    return std::move(return_grid);
}


int MazeToMap::get_grid_height_cells(){
    return m_board_height_pixels;
}

int MazeToMap::get_grid_width_cells(){
    return m_board_width_pixels; 
} 

double MazeToMap::get_resolution(){
    return m_resolution; 
}   


// // --- Helper for Visualization ---
// v