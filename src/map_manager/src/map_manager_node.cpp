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

using namespace std::chrono_literals;
using namespace std::placeholders;

class MapManagerNode : public rclcpp::Node
{
    public:
    MapManagerNode() : Node("map_manager")
    {
        // Bikin publisher, timer, dan service
        
        // publisher ke topic /map_state
        map_state_pub_ = this->create_publisher<custom_interface::msg::MapState>(
            "map_state", 
            10
        );
        
        // timer buat publish map state
        map_timer_ = this->create_wall_timer(
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

        // Inisialisasi map state
        // Misal 8x8 kaya di spesifikasi
        map_state_msg_.set__width(8);
        map_state_msg_.set__height(8);
        
        // Inisialisasi grid data
        // Set semua grid data jadi kosong
        map_state_msg_.set__grid_data(
            std::vector<int32_t>(map_state_msg_.width * map_state_msg_.height, KOSONG)
        );
        
        // Inisialisasi path numbers
        map_state_msg_.set__path_numbers(
            std::vector<int32_t>(map_state_msg_.width * map_state_msg_.height, 0)
        );
        
        RCLCPP_INFO(this->get_logger(), "Keadaan peta berhasil diinisialisasi, Cik!");
    }

    private:
        // Pendefinisian atribut class
        rclcpp::Publisher<custom_interface::msg::MapState>::SharedPtr map_state_pub_;
        rclcpp::TimerBase::SharedPtr map_timer_; // Delay buat publish map state
        rclcpp::Service<custom_interface::srv::AddObstacle>::SharedPtr add_obstacle_srv_;
        rclcpp::Service<custom_interface::srv::AddVictim>::SharedPtr add_victim_srv_;
        custom_interface::msg::MapState map_state_msg_; // Data map state

        // Fungsi buat publish map state
        void publish_map_state()
        {
            map_state_pub_->publish(map_state_msg_);
            RCLCPP_INFO(
                this->get_logger(), 
                "Berhasil publish keadaan dari peta (%d x %d), Cik!", 
                map_state_msg_.width, 
                map_state_msg_.height
            );
        }

        // Fungsi buat nambahin rintangan di map
        void add_obstacle_callback(
            const std::shared_ptr<custom_interface::srv::AddObstacle::Request> request,
            std::shared_ptr<custom_interface::srv::AddObstacle::Response> response
            )
        {
            // Early return kalo koordinat rintangan invalid yakni di luar ukuran map
            if (request->x < 0 || request->x >= map_state_msg_.width || request->y < 0 || request->y >= map_state_msg_.height)
            {
                response->success = false;
                response->message = "Koordinat rintangan gak valid, Cik!";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            
            // Hitung index dari koordinat (request->x, request->y) di grid data (indeks: w*y + x)
            int idx = map_state_msg_.width * request->y + request->x;

            // Early return kalo udah ada rintangan atau korban di koordinat (request->x, request->y)
            if (map_state_msg_.grid_data[idx] == OBSTACLE || map_state_msg_.grid_data[idx] == VICTIM)
            {
                response->success = false;
                response->message = "Di koordinat (" + std::to_string(request->x) + ", " + std::to_string(request->y) +  ") udah ada rintangan atau korban, Cik!";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            // Koordinat rintangan valid -> nambahin rintangan
            map_state_msg_.grid_data[idx] = OBSTACLE;
            
            // Response berhasil
            response->success = true;
            response->message = "Rintangan berhasil ditambahkan di koordinat (" + std::to_string(request->x) + ", " + std::to_string(request->y) + "), Cik!";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        }

        // Fungsi buat nambahin korban di map
        void add_victim_callback(
            const std::shared_ptr<custom_interface::srv::AddVictim::Request> request,
            std::shared_ptr<custom_interface::srv::AddVictim::Response> response
            )
        {
            // Early return kalo koordinat korban invalid yakni di luar ukuran map
            if (request->x < 0 || request->x >= map_state_msg_.width || request->y < 0 || request->y >= map_state_msg_.height)
            {
                response->success = false;
                response->message = "Koordinat korban gak valid, Cik!";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            
            // Hitung index dari koordinat (request->x, request->y) di grid data (indeks: w*y + x)
            int idx = map_state_msg_.width * request->y + request->x;

            // Early return kalo udah ada rintangan atau korban di koordinat (request->x, request->y)
            if (map_state_msg_.grid_data[idx] == OBSTACLE || map_state_msg_.grid_data[idx] == VICTIM)
            {
                response->success = false;
                response->message = "Di koordinat (" + std::to_string(request->x) + ", " + std::to_string(request->y) +  ") udah ada rintangan atau korban, Cik!";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            // Koordinat rintangan valid -> nambahin rintangan
            map_state_msg_.grid_data[idx] = VICTIM;
            
            // Response berhasil
            response->success = true;
            response->message = "Rintangan berhasil ditambahkan di koordinat (" + std::to_string(request->x) + ", " + std::to_string(request->y) + "), Cik!";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}