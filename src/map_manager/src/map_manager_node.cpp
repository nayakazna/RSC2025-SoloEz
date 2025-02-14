#define KOSONG 0
#define OBSTACLE -1
#define VICTIM -2

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interface/msg/map_state.hpp"
#include "custom_interface/srv/add_obstacle.hpp"
#include "custom_interface/srv/add_victim.hpp"
#include "map_state.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class MapManagerNode : public rclcpp::Node
{
    public:
    MapManagerNode() : Node("map_manager"), map_state_(8, 8)
    {
        // Bikin publisher, timer, dan service
        // publisher ke topic /map_state
        this->map_state_pub_ = this->create_publisher<custom_interface::msg::MapState>(
            "map_state", 
            10
        );
        
        // timer buat publish map state
        this->map_timer_ = this->create_wall_timer(
            500ms, 
            std::bind(&MapManagerNode::publish_map_state, this)
        );
        
        // service /add_obstacle
        // Nambahin rintangan di map
        add_obstacle_srv_ = this->create_service<custom_interface::srv::AddObstacle>(
            "add_obstacle", 
            std::bind(
                &MapManagerNode::add_obstacle_callback, // ini didefinisiin di private, di bawah
                this, 
                _1, 
                _2
            )
        );
        
        // service /add_victim
        // Nambahin korban yg harus diselametin di map
        add_victim_srv_ = this->create_service<custom_interface::srv::AddVictim>(
            "add_victim", 
            std::bind(
                &MapManagerNode::add_victim_callback,  // ini didefinisiin di private, di bawah
                this, 
                _1, 
                _2
            )
        );
        
        RCLCPP_INFO(this->get_logger(), "Node map_state berhasil diinisialisasi, Cik!");
    }

    private:
        // Pendefinisian atribut class
        rclcpp::Publisher<custom_interface::msg::MapState>::SharedPtr map_state_pub_;
        rclcpp::TimerBase::SharedPtr map_timer_;
        rclcpp::Service<custom_interface::srv::AddObstacle>::SharedPtr add_obstacle_srv_;
        rclcpp::Service<custom_interface::srv::AddVictim>::SharedPtr add_victim_srv_;
        MapState map_state_;


        // Fungsi buat publish map state
        void publish_map_state() {
            custom_interface::msg::MapState msg;
            msg.set__width(map_state_.getWidth());
            msg.set__height(map_state_.getHeight());
            msg.set__grid_data(map_state_.getGridData());
            std::string grid_str = map_state_.GridToArray();

            this->map_state_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "\n");
            RCLCPP_INFO(this->get_logger(), "==== MapState ====");
            RCLCPP_INFO(this->get_logger(), "\n%s", grid_str.c_str());
        }

        // Fungsi buat nambahin rintangan di map
        void add_obstacle_callback(
            const std::shared_ptr<custom_interface::srv::AddObstacle::Request> request,
            std::shared_ptr<custom_interface::srv::AddObstacle::Response> response
        ) {
            // Early return kalo koordinat rintangan invalid yakni di luar ukuran map
            if (request->x < 0 || request->x >= map_state_.getWidth() || request->y < 0 || request->y >= map_state_.getHeight()) {
                response->success = false;
                response->message = "Koordinat rintangan gak valid, Cik!";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            
            // Early return kalo udah ada rintangan atau korban di koordinat (request->x, request->y)
            if (map_state_.getGridValue(request->x, request->y) == OBSTACLE || 
                map_state_.getGridValue(request->x, request->y) == VICTIM) {
                response->success = false;
                response->message = "Di koordinat (" + std::to_string(request->x) + ", " + std::to_string(request->y) +  ") udah ada rintangan atau korban, Cik!";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            // Koordinat rintangan valid -> nambahin rintangan
            map_state_.setObstacle(request->x, request->y);
            
            // Response berhasil
            response->success = true;
            response->message = "Rintangan berhasil ditambahkan di koordinat (" + std::to_string(request->x) + ", " + std::to_string(request->y) + "), Cik!";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        }

        // Fungsi buat nambahin korban di map
        void add_victim_callback(
            const std::shared_ptr<custom_interface::srv::AddVictim::Request> request,
            std::shared_ptr<custom_interface::srv::AddVictim::Response> response
        ) {
            // Early return kalo koordinat korban invalid yakni di luar ukuran map
            if (request->x < 0 || request->x >= map_state_.getWidth() || request->y < 0 || request->y >= map_state_.getHeight()) {
                response->success = false;
                response->message = "Koordinat korban gak valid, Cik!";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            
            // Early return kalo udah ada rintangan atau korban di koordinat (request->x, request->y)
            if (map_state_.getGridValue(request->x, request->y) == OBSTACLE || 
                map_state_.getGridValue(request->x, request->y) == VICTIM) {
                response->success = false;
                response->message = "Di koordinat (" + std::to_string(request->x) + ", " + std::to_string(request->y) +  ") udah ada rintangan atau korban, Cik!";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            // Koordinat rintangan valid -> nambahin rintangan
            map_state_.setVictim(request->x, request->y);
            
            // Response berhasil
            response->success = true;
            response->message = "Korban berhasil ditambahkan di koordinat (" + std::to_string(request->x) + ", " + std::to_string(request->y) + "), Cik!";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}