#include <fstream>
#include <vector>
#include <string>
#include "maze_generator/utils.hpp"
#include <sstream>


struct MetaData{
    int board_width;
    int board_height; 
    double board_res; 
    double map_resolution; 
    int num_walls; 
    double wall_thickness; 
    std::string files_path; 
    std::vector<Wall> walls; 
};

class MazeWriter {
public:
    MazeWriter();
    void writeFile(const MetaData &meta_data);

private:
    std::string createWallModel(const std::string& name, const Wall& w);
    void createMetaData(const std::string& name, const MetaData &meta_data);
    std::vector<Wall> m_walls;
};
