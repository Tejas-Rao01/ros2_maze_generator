#include "maze_generator/maze_writer.hpp"
#include <iostream> 
#include <fstream>
#include <sstream>
#include <cmath>

MazeWriter::MazeWriter(){}

void MazeWriter::writeFile(const MetaData &meta_data) {
    std::string world_file = meta_data.files_path + ".world";
    std::string metadata_file = meta_data.files_path + "_metadata.json";
    
    std::ofstream ofs(world_file);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open world file for writing: " << world_file << std::endl;
        return;
    }
    ofs << "<?xml version=\"1.0\" ?>\n<sdf version=\"1.6\">\n<world name=\"maze_world\">\n";
    ofs << "  <scene>\n";
    ofs << "    <ambient>0.4 0.4 0.4 1</ambient>\n";
    ofs << "    <background>0.7 0.7 0.7 1</background>\n";
    ofs << "  </scene>\n";
    ofs << "  <light name=\"sun\" type=\"directional\">\n";
    ofs << "    <cast_shadows>true</cast_shadows>\n";
    ofs << "    <pose>0 0 10 0 0 0</pose>\n";
    ofs << "    <diffuse>0.8 0.8 0.8 1</diffuse>\n";
    ofs << "    <specular>0.2 0.2 0.2 1</specular>\n";
    ofs << "    <direction>-0.5 0.1 -0.9</direction>\n";
    ofs << "  </light>\n";
    ofs << "  <model name=\"ground_plane\">\n";
    ofs << "    <static>true</static>\n";
    ofs << "    <link name=\"link\">\n";
    ofs << "      <collision name=\"collision\">\n";
    ofs << "        <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>\n";
    ofs << "      </collision>\n";
    ofs << "      <visual name=\"visual\">\n";
    ofs << "        <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>\n";
    ofs << "        <material>\n";
    ofs << "          <ambient>0.8 0.8 0.8 1</ambient>\n";
    ofs << "          <diffuse>0.8 0.8 0.8 1</diffuse>\n";
    ofs << "          <specular>0.1 0.1 0.1 1</specular>\n";
    ofs << "        </material>\n";
    ofs << "      </visual>\n";
    ofs << "    </link>\n";
    ofs << "  </model>\n";
    int count = 0;
    for (const auto& w : meta_data.walls) {
        ofs << createWallModel("wall_" + std::to_string(count++), w);
    }

    ofs << "</world>\n</sdf>\n";
    ofs.close();

    createMetaData(metadata_file, meta_data);
}

std::string MazeWriter::createWallModel(const std::string& name, const Wall& w) {
    double wall_thickness = 0.2; 
    double wall_height = 1.0;

    double size_x = std::abs(w.end_x - w.start_x);
    double size_y = std::abs(w.end_y - w.start_y);

    // Calculate center pose
    double pos_x = (w.start_x + w.end_x) / 2.0;
    double pos_y = (w.start_y + w.end_y) / 2.0; 

    size_x = std::max(size_x, wall_thickness);
    size_y = std::max(size_y, wall_thickness);

    std::ostringstream oss;
    oss << "  <model name=\"" << name << "\">\n"
        << "    <static>true</static>\n"
        << "    <pose>" << pos_x << " " << pos_y << " " << (wall_height / 2.0) << " 0 0 0</pose>\n"
        << "    <link name=\"link\">\n"
        << "      <collision name=\"collision\">\n"
        << "        <geometry><box><size>" << size_x << " " << size_y << " " << wall_height << "</size></box></geometry>\n"
        << "      </collision>\n"
        << "      <visual name=\"visual\">\n"
        << "        <geometry><box><size>" << size_x << " " << size_y << " " << wall_height << "</size></box></geometry>\n"
        << "        <material><ambient>0.7 0.7 0.7 1</ambient></material>\n"
        << "      </visual>\n" 
        << "    </link>\n"
        << "  </model>\n";
    return oss.str();
}

void MazeWriter::createMetaData(const std::string& name, const MetaData &meta_data){
    std::ofstream ofs(name);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open metadata file for writing: " << name << std::endl;
        return;
    }

    ofs << "{\n";
    ofs << "  \"board_width\": " << meta_data.board_width << ",\n";
    ofs << "  \"board_height\": " << meta_data.board_height << ",\n";
    ofs << "  \"board_res\": " << meta_data.board_res << ",\n";
    ofs << "  \"map_resolution\": " << meta_data.map_resolution << ",\n";
    ofs << "  \"num_walls\": " << meta_data.walls.size() << ",\n";
    ofs << "  \"wall_thickness\": " << meta_data.wall_thickness << ",\n";
    ofs << "  \"walls\": [\n";

    for (size_t i = 0; i < meta_data.walls.size(); ++i) {
        const Wall& w = meta_data.walls[i];
        ofs << "    [" << w.start_x << ", " << w.start_y << ", "
            << w.end_x << ", " << w.end_y << "]";
        if (i != meta_data.walls.size() - 1) {
            ofs << ",";
        }
        ofs << "\n";
    }
    ofs << "  ]\n";
    ofs << "}\n";
}
