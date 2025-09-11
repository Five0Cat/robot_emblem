#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
std::string yaml_path = "src/robot_emblem/maps/demo_map.yaml";
struct Map{
    int width;
    int height ;
    double resolution;
    double origin_x,origin_y;
    std::vector<int> data;
};
class MaploaderNode : public rclcpp::Node{
    public:
        MaploaderNode():rclcpp::Node("map_loader"){
        map = loadMapFromYaml(yaml_path);
        
        map_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/map_markers", 10);

        // 定时发布
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MaploaderNode::publishMap, this)
        );
        }
    private:
        Map loadMapFromYaml(const std::string& yaml_path){
            Map map;
            std::ifstream fin(yaml_path);
            YAML::Node config = YAML::Load(fin);
        
            map.resolution = config["resolution"].as<double>();
            map.origin_x = config["origin"][0].as<double>();
            map.origin_y = config["origin"][1].as<double>();
            map.width = config["width"].as<int>();
            map.height = config["height"].as<int>();
                
            const auto& data = config["data"];
            for (const auto& value : data) {
                map.data.push_back(value.as<int>());
        }
            return map;
        }

        void publishMap() {
        visualization_msgs::msg::MarkerArray marker_array;

        for (int y = 0; y < map.height; ++y) {
            for (int x = 0; x < map.width; ++x) {
                int idx = y * map.width + x;
                int terrain = map.data[idx];

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->now();
                marker.id = idx;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                // 设置位置
                marker.pose.position.x = map.origin_x + (x+0.5)* map.resolution;
                marker.pose.position.y = map.origin_y + (y+0.5) * map.resolution;
                marker.pose.position.z = 0.0;

                // 设置尺寸
                marker.scale.x = map.resolution;
                marker.scale.y = map.resolution;
                marker.scale.z = 0.1;

                // 设置颜色
                std_msgs::msg::ColorRGBA color;
                switch (terrain) {
                    case 0: // free
                        color.r = 0.5f; color.g = 0.5f; color.b = 0.5f; color.a = 1.0f;
                        break;
                    case 1: // tree
                        color.r = 0.0f; color.g = 1.0f; color.b = 0.0f; color.a = 1.0f;
                        break;
                    case 2: // ice
                        color.r = 0.0f; color.g = 0.0f; color.b = 1.0f; color.a = 1.0f;
                        break;
                    default:
                        color.r = 0.0f; color.g = 0.0f; color.b = 0.0f; color.a = 0.0f;
                        break;
                }
                marker.color = color;

                // 添加marker
                marker_array.markers.push_back(marker);
        
            }
        }
        // === add grid lines ===
        visualization_msgs::msg::Marker grid;
        grid.header.frame_id = "map";
        grid.header.stamp = this->now();
        grid.ns = "grid_lines";
        grid.id = 999999; // 确保唯一
        grid.type = visualization_msgs::msg::Marker::LINE_LIST;
        grid.action = visualization_msgs::msg::Marker::ADD;

        // 线宽（米）
        grid.scale.x = std::max(0.01, map.resolution * 0.05); // 5% 的格宽，最低 1cm
        // 颜色（深灰/黑，半透明）
        grid.color.r = 0.0f; grid.color.g = 0.0f; grid.color.b = 0.0f; grid.color.a = 0.6f;

        // 放在方块之上，避免 z-fighting（你的方块厚度是 0.05）
        const double z = 0.051;

        // 垂直线：i = 0..width
        for (int i = 0; i <= map.width; ++i) {
        double x = map.origin_x + i * map.resolution;
        geometry_msgs::msg::Point p0, p1;
        p0.x = x; p0.y = map.origin_y;                    p0.z = z;
        p1.x = x; p1.y = map.origin_y + map.height * map.resolution; p1.z = z;
        grid.points.push_back(p0);
        grid.points.push_back(p1);
        }

        // 水平线：j = 0..height
        for (int j = 0; j <= map.height; ++j) {
        double y = map.origin_y + j * map.resolution;
        geometry_msgs::msg::Point p0, p1;
        p0.x = map.origin_x;                    p0.y = y; p0.z = z;
        p1.x = map.origin_x + map.width * map.resolution; p1.y = y; p1.z = z;
        grid.points.push_back(p0);
        grid.points.push_back(p1);
        }

        marker_array.markers.push_back(grid);
        map_pub_->publish(marker_array);

    }
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    Map map; 
};


int main(int argc , char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MaploaderNode>());
    rclcpp::shutdown();
    return 0;
}