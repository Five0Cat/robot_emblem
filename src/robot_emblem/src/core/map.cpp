#include "robot_emblem/core/game.hpp"
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
namespace re{
namespace io{

static Tile tile_from_int(int v){
    switch(v){
        case 0 : return Tile::Free;
        case 1 : return Tile::Tree;
        case 2 : return Tile::Ice;
        default: return Tile::Free;
    }
}

MapGrid load_map_yaml(const std::string& path){
  YAML::Node root = YAML::LoadFile(path);
  if(!root["width"] || !root["height"] || !root["tiles"]) {
    throw std::runtime_error("Invalid map yaml: require width,height,tiles");
  }
  MapGrid m;
  m.w = root["width"].as<int>();
  m.h = root["height"].as<int>();

  const auto& t = root["tiles"];
  if(!t.IsSequence() || (int)t.size() != m.w * m.h) {
    throw std::runtime_error("tiles size must be width*height (row-major)");
  }
  m.tiles.reserve(m.w * m.h);
  for(int i=0;i<m.w*m.h;i++){
    int vi = t[i].as<int>();
    m.tiles.push_back(tile_from_int(vi));
  }
  return m;
}
} //name space io

}//namespace re

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("map_node"), "Map node started!");

    // TODO: test your load_map_yaml here if you want
    // auto map = re::load_map_yaml("/path/to/map.yaml");

    rclcpp::shutdown();
    return 0;
}