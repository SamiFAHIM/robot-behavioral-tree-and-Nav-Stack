#include "nav2_custom_behavior_tree/behavior_tree_engine.hpp"
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/int32.hpp>
#include <memory>

#define blue 0
#define yellow 1

class MatchNode : public rclcpp::Node
{
public:
    MatchNode()
        : Node("match_node"), tirette_pulled(0)
    {
        tirette_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "ctrl_panel/tirette", 10, std::bind(&MatchNode::tirette_callback, this, std::placeholders::_1));

        team_color_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "ctrl_panel/right_button", 10, std::bind(&MatchNode::team_color_callback, this, std::placeholders::_1));

        clear_global_costmap_srv_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
        clear_local_costmap_srv_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");

        bt_ = std::make_unique<nav2_custom_behavior_tree::BehaviorTreeEngine>();

        int try_count = 0;
        bool bt_init_successfully = false;
        while (try_count < 10 && !bt_init_successfully)
        {
            bt_init_successfully = bt_->init();
            try_count++;
        }
        if (!bt_init_successfully)
        {
            rclcpp::shutdown();
            RCLCPP_ERROR(get_logger(), "Failed to initialize Behavior Tree !");
        }
    }

    void start_match()
    {
        RCLCPP_INFO(get_logger(), "Match started !");
        auto req1 = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        this->clear_global_costmap_srv_->async_send_request(req1);
        auto req2 = std::make_shared<nav2_msgs::srv::ClearEntireCostmap_Request>();
        this->clear_local_costmap_srv_->async_send_request(req2);
        bt_->run();
    }

    int tirette_pulled;

private:
    void tirette_callback(const std_msgs::msg::Int32 &msg)
    {
        tirette_pulled = msg.data;
    }

    void team_color_callback(const std_msgs::msg::Int32 &msg)
    {
        if (msg.data && last_team_color_button_state == 0)
        {
            team_color == blue ? team_color = yellow : team_color = blue;
            RCLCPP_INFO(get_logger(), "Team color changed to %s", team_color == blue ? "blue" : "yellow");
            team_color == blue ? bt_->init({}, bt_->default_blue_bt_file_path) : bt_->init({}, bt_->default_yellow_bt_file_path);
        }
        last_team_color_button_state = msg.data;
    }

    std::unique_ptr<nav2_custom_behavior_tree::BehaviorTreeEngine> bt_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tirette_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr team_color_subscription_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_costmap_srv_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_costmap_srv_;

    int last_team_color_button_state = 0;
    int team_color = blue;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto match_node = std::make_shared<MatchNode>();
    exec.add_node(match_node);

    RCLCPP_INFO(match_node->get_logger(), "Waiting for tirette to be pulled ...");
    while (rclcpp::ok() && match_node->tirette_pulled == 0)
    {
        exec.spin_once(std::chrono::milliseconds(100));
    }

    if (rclcpp::ok())
    {
        match_node->start_match();
    }
    rclcpp::shutdown();
    return 0;
}
