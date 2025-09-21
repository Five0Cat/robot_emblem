#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>
#include <queue>
#include <set>
#include <string>


struct Cell{
    int ix,iy,cost;
};

struct MapData {
  int width = 0;
  int height = 0;
  double resolution = 1.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
};

class UnitManagerNode : public rclcpp::Node{
    public : 
        UnitManagerNode() : Node("unit_manager_node"){
            this->declare_parameter<std::string>("map_yaml","maps/demo_map.yaml");
            this->declare_parameter<int>("unit_ix",0);
            this->declare_parameter<int>("unit_iy",0);
            // unit_name_ = "unit/alpha";
            unit_name_ = "base_link";

            std::string yaml_path = this->get_parameter("map_yaml").as_string();
            if (!loadMap(yaml_path, map_)) {
            RCLCPP_FATAL(get_logger(), "Failed to load map yaml: %s", yaml_path.c_str());
            rclcpp::shutdown();
            return;}

            ix_ = this->get_parameter("unit_ix").as_int();
            iy_ = this->get_parameter("unit_iy").as_int();
            goal_ix_ = ix_;
            goal_iy_ = iy_;

            marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/unit_marker",1);
            auto qos = rclcpp::QoS(1).transient_local();  // 带记忆 QoS，新订阅者也能看到
            move_area_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/reachable", qos);
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            clicked_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&UnitManagerNode::onClicked, this, std::placeholders::_1));

            // 定时器：发布 TF + 单位 Marker + 执行一步移动
            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(80),
            std::bind(&UnitManagerNode::onTimer, this));

        }
    private :
        bool loadMap(const std::string& path, MapData& out) {
            try {
            YAML::Node config = YAML::LoadFile(path);
            out.resolution = config["resolution"].as<double>();
            out.origin_x   = config["origin"][0].as<double>();
            out.origin_y   = config["origin"][1].as<double>();
            out.width      = config["width"].as<int>();
            out.height     = config["height"].as<int>();
            return true;
            } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "YAML load error: %s", e.what());
            return false;
            }
        }
        //world -> local
        bool worldToIdx(double x,double y,int& ix , int & iy) const{
            ix=static_cast<int>(std::floor(x-map_.origin_x)/map_.resolution);
            iy=static_cast<int>(std::floor(y-map_.origin_y)/map_.resolution);
            return(ix>=0 && ix<map_.width && iy>=0 &&iy<map_.height);
        }
        //local -> world
        void idxToWorld(int ix, int iy, double& x, double& y){
             x = map_.origin_x + (ix + 0.5) * map_.resolution;
             y = map_.origin_y + (iy + 0.5) * map_.resolution; //+0.5 means locate at middle of each grid
        }
        void stepTowardsGoal(){
            if(ix_ == goal_ix_ && iy_ == goal_iy_) return;
            if(ix_!=goal_ix_) ix_+=(goal_ix_ > ix_)? 1:-1;
           else if(iy_!=goal_iy_) iy_ +=(goal_iy_ > iy_)? 1:-1;
        }

        void publishTF() {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->now();
            t.header.frame_id = "map";
            t.child_frame_id  = "base_link";

            double x, y;
            idxToWorld(ix_, iy_, x, y);
            t.transform.translation.x = x;
            t.transform.translation.y = y;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(t);
        }
        void publishMarker() {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = this->now();
            m.ns = "units";
            m.id = 1;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;

            double x, y;
            idxToWorld(ix_, iy_, x, y);
            m.pose.position.x = x;
            m.pose.position.y = y;
            m.pose.position.z = 0.05; // 抬一点
            m.pose.orientation.w = 1.0;

            m.scale.x = map_.resolution * 0.7;  // 直径略小于一格
            m.scale.y = map_.resolution * 0.7;
            m.scale.z = 0.1;

            // 颜色：黄色
            m.color.r = 1.0f; m.color.g = 0.85f; m.color.b = 0.2f; m.color.a = 1.0f;

            marker_pub_->publish(m);
     }

     void publishMoveArea(){
        //Bfs frontier
        std::queue<Cell> q;
        std::set<std::pair<int,int>> visited;

        q.push({ix_,iy_,0});
        visited.insert({ix_,iy_});

        visualization_msgs::msg::Marker reach, block;

        auto makeMarker = [&](int id, const std::string& ns,
                        float r, float g, float b, float a){
                                visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = this->now();
            m.ns = ns;
            m.id = id;
            m.type = visualization_msgs::msg::Marker::CUBE_LIST;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = map_.resolution * 0.95;
            m.scale.y = map_.resolution * 0.95;
            m.scale.z = 0.1;
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
            return m;
                        };
            reach = makeMarker(0, "reachable", 0.0f, 1.0f, 0.0f, 0.4f);  // green
            block = makeMarker(1, "blocked",   1.0f, 0.0f, 0.0f, 0.2f);  // red   
            //BFS loop
            while(!q.empty()){
                Cell cur = q.front(); q.pop();
                
                double wx,wy;
                idxToWorld(cur.ix,cur.iy,wx,wy);

                geometry_msgs::msg::Point p;
                p.x = wx; p.y = wy; p.z = 0.01;

                if(cur.cost <= move_points_)
                {
                    reach.points.push_back(p);
                    reachable_set_.insert({cur.ix, cur.iy});  // record grid index
                }else{
                    block.points.push_back(p);
                }
                if(cur.cost < move_points_){
                     // 4-connected Manhattan neighbors
                const int dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
                for(auto &d:dirs){
                    int nx = cur.ix + d[0];
                    int ny = cur.iy + d[1];
                    if( nx < 0 || ny < 0 || nx >= map_.width || ny >= map_.height) continue;
                    if (visited.count({nx,ny})) continue;

                    visited.insert({nx,ny});
                    q.push({nx, ny, cur.cost + 1});
                }

                }
            }
            visualization_msgs::msg::MarkerArray arr;
            arr.markers.push_back(reach);
            arr.markers.push_back(block);
            move_area_pub_->publish(arr);
     }


        void onClicked(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            int gx, gy;
            if (!worldToIdx(msg->point.x, msg->point.y, gx, gy)) {
            RCLCPP_WARN(get_logger(), "Clicked point out of map.");
            return;
            }
            
            if (!reachable_set_.count({gx, gy})) {
            RCLCPP_INFO(get_logger(), "Cell (%d,%d) NOT reachable this turn.", gx, gy);
            return;
            }

            goal_ix_ = gx;
            goal_iy_ = gy;
            RCLCPP_INFO(get_logger(), "Goal set to cell (%d,%d)", goal_ix_, goal_iy_);
        }
        void onTurnBegin() {
           publishMoveArea();
        }

        void onTimer() {
            if (!move_area_first_pub_) {
                    publishMoveArea();      // publish reachable area
                    // move_area_first_pub_ = true;
                }
            stepTowardsGoal();
            publishTF();
            publishMarker();
        }
        
        MapData map_;
        std::string unit_name_;
        int move_points_{4};   // move
        int ix_{0}, iy_{0};
        int goal_ix_{0}, goal_iy_{0};
        double cur_x_{0.0}, cur_y_{0.0};
        double linear_speed_;
        std::vector<std::vector<int>> dist_; 
        std::set<std::pair<int,int>> reachable_set_;
        bool move_area_first_pub_{false};  // 
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr move_area_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;
        
};
int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<UnitManagerNode>());
    rclcpp::shutdown();
    return 0;
}