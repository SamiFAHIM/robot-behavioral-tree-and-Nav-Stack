#include "nav2_custom_behavior_tree/behavior_tree_engine.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fstream>

namespace nav2_custom_behavior_tree
{
    BehaviorTreeEngine::BehaviorTreeEngine() : Node("bt_engine"),
                                               bt_plugins_loaded_successfully_(false), bt_tree_loaded_successfully_(false)
    {
        configure_parameters();
    }

    bool BehaviorTreeEngine::init(std::vector<std::string> plugins, std::string bt_file_path)
    {
        if (!bt_plugins_loaded_successfully_ && !load_plugins(plugins))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load plugins");
            return false;
        }

        rclcpp::sleep_for(std::chrono::seconds(5)); // wait for plugins to load
        if (!load_tree(bt_file_path))
        {
            return false;
        }

        if (use_groot_monitoring_ && !groot_started_)
        {
            groot_started_ = true;
            add_groot_monitoring();
        }

        return true;
    }

    void BehaviorTreeEngine::configure_parameters()
    {
        client_node_ = std::make_shared<rclcpp::Node>("bt_node");
        server_timeout_ = std::chrono::milliseconds(this->declare_parameter("server_timeout", 250));
        wait_for_service_timeout_ = std::chrono::milliseconds(this->declare_parameter("wait_for_service_timeout", 1000));
        bt_loop_duration_ = std::chrono::milliseconds(this->declare_parameter("bt_loop_duration", 250));
        // tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        default_yellow_bt_file_path = ament_index_cpp::get_package_share_directory("nav2_custom_behavior_tree") + "/behavior_trees/match_one_planter_yellow.xml";
        this->declare_parameter("default_yellow_bt_file_path", rclcpp::ParameterType::PARAMETER_STRING);

        default_blue_bt_file_path = ament_index_cpp::get_package_share_directory("nav2_custom_behavior_tree") + "/behavior_trees/match_one_planter_blue.xml";
        this->declare_parameter("default_blue_bt_file_path", rclcpp::ParameterType::PARAMETER_STRING);

        this->declare_parameter("plugins", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);

        // Groot
        use_groot_monitoring_ = this->declare_parameter("run_groot_monitoring", true);
        publisher_port_ = this->declare_parameter("publisher_port", 1666);
        server_port_ = this->declare_parameter("server_port", 1667);
        max_msg_per_second_ = this->declare_parameter("max_msg_per_second", 25);
    }

    bool BehaviorTreeEngine::load_plugins(std::vector<std::string> plugins)
    {
        try
        {
            if (plugins.empty())
            {
                plugins_ = this->get_parameter("plugins").as_string_array();
            }
            RCLCPP_INFO(this->get_logger(), "Loading plugins ...");
            for (const auto &p : plugins_)
            {
                factory_.registerFromPlugin(BT::SharedLibrary::getOSName(p));
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception when loading BT: %s", e.what());
            bt_plugins_loaded_successfully_ = false;
            return false;
        }
        bt_plugins_loaded_successfully_ = true;
        RCLCPP_INFO(this->get_logger(), "Plugins loaded successfully !");
        return true;
    }

    bool BehaviorTreeEngine::load_tree(std::string bt_file_path)
    {
        // Empty filename is default for backward compatibility
        if (bt_file_path.empty())
        {
            bt_file_path = default_blue_bt_file_path;
        }

        RCLCPP_INFO(this->get_logger(), "Loading tree from file: %s", bt_file_path.c_str());

        // Read the input BT XML from the specified file into a string
        std::ifstream xml_file(bt_file_path);

        if (!xml_file.good())
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't open input XML file: %s", bt_file_path.c_str());
            bt_tree_loaded_successfully_ = false;
            return false;
        }

        blackboard_ = BT::Blackboard::create();
        blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);
        blackboard_->set<std::chrono::milliseconds>("server_timeout", server_timeout_);
        blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout_);
        blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
        // blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);

        try
        {
            tree_ = std::make_shared<BT::Tree>(factory_.createTreeFromFile(bt_file_path, blackboard_));
            for (auto &blackboard : tree_.get()->blackboard_stack)
            {
                blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_);
                blackboard->set<std::chrono::milliseconds>("server_timeout", server_timeout_);
                blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
                blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", wait_for_service_timeout_);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception when loading BT: %s", e.what());
            bt_tree_loaded_successfully_ = false;
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Tree loaded successfully !");
        bt_tree_loaded_successfully_ = true;
        return true;
    }

    void BehaviorTreeEngine::add_groot_monitoring()
    {
        groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
            *tree_, max_msg_per_second_, publisher_port_, server_port_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Groot monitoring enabled with server port [" << server_port_ << "] and publisher port [" << publisher_port_ << "]");
    }

    BT::NodeStatus BehaviorTreeEngine::run()
    {
        if (!bt_plugins_loaded_successfully_ || !bt_tree_loaded_successfully_)
        {
            RCLCPP_ERROR(this->get_logger(), "Behavior Tree failed to initialize");
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(this->get_logger(), "Running Behavior Tree ...");
        rclcpp::WallRate loopRate(bt_loop_duration_);
        BT::NodeStatus result = BT::NodeStatus::RUNNING;

        // Loop until something happens with ROS or the node completes
        try
        {
            start_time_ = this->now().seconds();
            while (rclcpp::ok() && result == BT::NodeStatus::RUNNING)
            {
                result = tree_->tickRoot();
                if (!loopRate.sleep())
                {
                    RCLCPP_WARN(
                        rclcpp::get_logger("BehaviorTreeEngine"),
                        "Behavior Tree tick rate %0.2f was exceeded!",
                        1.0 / (loopRate.period().count() * 1.0e-9));
                }

                if (this->now().seconds() - start_time_ >= 89)
                {
                    RCLCPP_INFO(this->get_logger(), "Match ended");
                    halt();
                    return BT::NodeStatus::FAILURE;
                }
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("BehaviorTreeEngine"),
                "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
            return BT::NodeStatus::FAILURE;
        }

        if (result == BT::NodeStatus::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Behavior Tree completed successfully");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Behavior Tree completed with failure");
        }

        return result;
    }

    void BehaviorTreeEngine::halt()
    {
        RCLCPP_INFO(this->get_logger(), "Halting Behavior Tree ...");
        tree_->haltTree();
        rclcpp::shutdown();
    }

} // namespace nav2_custom_behavior_tree
