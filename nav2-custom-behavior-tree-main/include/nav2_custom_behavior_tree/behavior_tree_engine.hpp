#ifndef NAV2_CUSTOM_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define NAV2_CUSTOM_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_ros/buffer.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

namespace nav2_custom_behavior_tree
{

    class BehaviorTreeEngine : public rclcpp::Node
    {
    public:
        BehaviorTreeEngine();
        bool init(std::vector<std::string> plugins = {}, std::string bt_file_path = "");
        BT::NodeStatus run();
        void halt();

        std::string default_yellow_bt_file_path;
        std::string default_blue_bt_file_path;

    private:
        void configure_parameters();
        bool load_plugins(std::vector<std::string> plugins);
        bool load_tree(std::string bt_file_path);
        void add_groot_monitoring();

        // Behavior Tree
        BT::BehaviorTreeFactory factory_;
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<BT::Tree> tree_;

        std::vector<std::string> plugins_;
        bool bt_plugins_loaded_successfully_;
        bool bt_tree_loaded_successfully_;

        // ROS Parameters
        std::shared_ptr<rclcpp::Node> client_node_;
        std::chrono::milliseconds server_timeout_;
        std::chrono::milliseconds wait_for_service_timeout_;
        std::chrono::milliseconds bt_loop_duration_;
        // std::shared_ptr<tf2_ros::Buffer> tf_;
        double start_time_;

        // Groot monitoring
        std::unique_ptr<BT::PublisherZMQ> groot_monitor_;
        bool use_groot_monitoring_;
        uint16_t publisher_port_, server_port_, max_msg_per_second_;
        bool groot_started_;
    };

} // namespace nav2_custom_behavior_tree

#endif // NAV2_CUSTOM_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
