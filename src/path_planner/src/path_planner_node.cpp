#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "custom_interface/msg/map_state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class PathPlannerNode : public rclcpp::Node {
    public:
        PathPlannerNode() : Node("path_planner_node")
        {
            // subscriber ke topic /map_state biar dapet keadaan peta terbaru
            map_state_sub_ = this->create_subscription<custom_interface::msg::MapState>(
                "map_state", 
                10, 
                std::bind(
                    &PathPlannerNode::map_state_callback, 
                    this, 
                    _1
                )
            );

            // publisher ke topic /path_visualization
            path_visualization_pub_ = this->create_publisher<std_msgs::msg::String>("path_visualization", 10);

            // timer buat publish path
            timer_ = this->create_wall_timer(500ms, std::bind(&PathPlannerNode::publish_path, this));

            RCLCPP_INFO(this->get_logger(), "Node path_planner berhasil diinisialisasi, Cik!");
        }

    private:
        rclcpp::Subscription<custom_interface::msg::MapState>::SharedPtr map_state_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_visualization_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Method callback buat subscriber ke topic /map_state
        void map_state_callback(const custom_interface::msg::MapState::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received map state: width=%d, height=%d", msg->width, msg->height);
            // TODO: Proses map state sama bikin path 
        }

        // Method buat publish path visualization ke topic /path_visualization
        void publish_path() {
            auto path_msg = std_msgs::msg::String();
            path_msg.data = "Dummy path visualization";
            path_visualization_pub_->publish(path_msg);
            RCLCPP_INFO(this->get_logger(), "Published dummy path visualization");
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
