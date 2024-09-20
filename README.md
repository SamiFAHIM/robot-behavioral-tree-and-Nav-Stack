# robot-behavioral-tree-and-Nav-Stack
### Project Overview
This project implements a Behavior Tree (BT) Engine designed to facilitate the control and monitoring of autonomous systems using Behavior Trees in ROS2. Behavior Trees provide a modular and flexible approach to decision-making in robotic systems. The engine loads and executes BTs defined in XML files and manages plugins for custom behavior nodes.

### Features
- Plugin Management: The engine can load user-defined plugins to extend the behavior tree's capabilities.
- Behavior Tree Loading: Behavior trees are loaded from XML files, with support for parameterized configuration.
- Monitoring with Groot: Optional integration with Groot, a tool for visualizing and monitoring behavior trees, to provide real-time insights into tree execution.
- ROS2 Node: This engine runs as a ROS2 node, utilizing the ROS2 ecosystem for communication, parameter handling, and logging.
### Main Functionality
- Plugin Loading: Dynamically loads behavior plugins, enabling users to extend the set of available behavior tree nodes.
- Tree Execution: Executes behavior trees defined in XML files. The system supports different configurations for task allocation and sequencing.
- Error Handling: Includes error handling for plugin loading and tree execution, ensuring robust performance.
### Dependencies
ROS2 Humble or later
rclcpp (ROS2 C++ Client Library)
Custom plugins (if any)
Groot (optional for monitoring)
### Usage
Configure Parameters: Define the parameters like bt_loop_duration, server_timeout, and paths to behavior tree XML files.
Initialize Engine: Call the init() function, passing in the required plugins and the path to the behavior tree file.
Run the Behavior Tree: Once the plugins and the tree are successfully loaded, the behavior tree engine will begin executing the tree's tasks.
### Files
behavior_tree_engine.cpp: Contains the core logic for managing behavior trees, loading plugins, and running the engine as a ROS2 node.
XML Files: Define the structure of the behavior trees. These are referenced via file paths and loaded at runtime.
### Future Work
Add support for more complex error recovery strategies.
Extend monitoring capabilities with Groot for more detailed insights.
