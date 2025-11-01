#include "maze_generator/maze_writer.hpp"
#include <iostream> 

MazeWriter::MazeWriter(){}

void MazeWriter::writeFile(const MetaData &meta_data) {

        std::cout << "files path: " << meta_data.files_path << std::endl;

        std::string world_file = meta_data.files_path + ".world";
        std::string metadata_file = meta_data.files_path + "_metadata.json";
        std::ofstream ofs(world_file);
        ofs << "<?xml version=\"1.0\" ?>\n<sdf version=\"1.6\">\n<world name=\"maze_world\">\n";
        ofs << "  <include><uri>model://sun</uri></include>\n";
        ofs << "  <include><uri>model://ground_plane</uri></include>\n";

        int count = 0;
        for (auto& w : meta_data.walls) {
            ofs << createWallModel("wall_" + std::to_string(count++), w);
        }

        createMetaData(metadata_file, meta_data);

        ofs << "</world>\n</sdf>\n";
    }

std::string MazeWriter::createWallModel(const std::string& name, const Wall& w) {

        double wall_thickness = 0.2; 

        double size_x = std::abs(w.end_x - w.start_x);
        double size_y = std::abs(w.end_y - w.start_y);

        double start_x = (2 * w.start_x + size_x) / 2;
        double start_y = (2 * w.start_y + size_y) / 2; 


        size_x = std::max(size_x, wall_thickness);
        size_y = std::max(size_y, wall_thickness);

        std::ostringstream oss;
        oss << "  <model name=\"" << name << "\">\n"
            << "    <static>true</static>\n"
            << "    <pose>" << start_x << " " << start_y << " " << 0 << " 0 0 0</pose>\n"
            << "    <link name=\"link\">\n"
            << "      <collision name=\"collision\">\n"
            << "        <geometry><box><size>" << size_x << " " << size_y << " " << 1 << "</size></box></geometry>\n"
            << "      </collision>\n"
            << "      <visual name=\"visual\">\n"
            << "        <geometry><box><size>" << size_x << " " << size_y << " " << 1 << "</size></box></geometry>\n"
            << "        <material><ambient>0.7 0.7 0.7 1</ambient></material>\n"
            << "      </visual>\n" 
            << "    </link>\n"
            << "  </model>\n";
        return oss.str();
    }
void MazeWriter::createMetaData(const std::string& name, const MetaData &meta_data){
    std::ofstream ofs(name);
    ofs << "{\n";
    ofs << "  \"board_width\": " << meta_data.board_width << ",\n";
    ofs << "  \"board_height\": " << meta_data.board_height << ",\n";
    ofs << "  \"board_res\": " << meta_data.board_res << ",\n";
    ofs << "  \"map_resolution\": " << meta_data.map_resolution << ",\n";
    ofs << "  \"num_walls\": " << meta_data.walls.size() << ",\n";
    ofs << "  \"wall_thickness\": " << meta_data.wall_thickness << ",\n";
    ofs << "  \"walls\": [\n";


    std::cout << "board_width: " << meta_data.board_width << " board_height lauda: " << meta_data.board_height << std::endl;
    auto walls = meta_data.walls;
    for (size_t i = 0; i < walls.size(); ++i) {
        const Wall& w = walls[i];
        ofs << "    [" << w.start_x << ", " << w.start_y << ", "
            << w.end_x << ", " << w.end_y << "]";
        if (i != walls.size() - 1) {
            ofs << ",";
        }
        ofs << "\n";
    }
    ofs << "  ]\n";
    ofs << "}\n";

}


// int main(int argc, char **argv){
//     std::vector<Wall> walls; 
//     walls.push_back({0, 0, 5, 0});
//     walls.push_back({0, 0, 0, 5});
//     MazeWriter writer(walls);
//     writer.writeFile();
// }