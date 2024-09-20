from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


from rclpy.node import Node
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import RegisterEventHandler
from launch.events.process import ProcessStarted
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():

    # Get the launch directory
    bt_pkg_dir = get_package_share_directory('nav2_custom_behavior_tree')

    bt_params = os.path.join(
        bt_pkg_dir,
        'config',
        'params.yaml'
    )
    default_yellow_bt_file_path = os.path.join(
        bt_pkg_dir,
        'behavior_trees',
        'match_one_planter_yellow.xml'
    )

    default_blue_bt_file_path = os.path.join(
        bt_pkg_dir,
        'behavior_trees',
        'match_one_planter_blue.xml'
    )

    beacon_present = LaunchConfiguration('beacon_present')

    opponent_pos = LaunchConfiguration('opponent_pos')

    configured_params = RewrittenYaml(
        source_file=bt_params,
        root_key='',
        param_rewrites={'default_yellow_bt_file_path': default_yellow_bt_file_path,
                        "default_blue_bt_file_path": default_blue_bt_file_path},
        convert_types=True)

    nav_utils_node = Node(
        package='robot_controller',
        executable='nav_utils',
        name='nav_utils',
        output='screen',
    )

    table_controller_node = Node(
        package='robot_controller',
        executable='table_controller',
        name='table_controller',
        output='screen',
        parameters=[{'beacon_present': beacon_present}],
    )
    match_node = Node(
        package="nav2_custom_behavior_tree",
        executable="nav2_custom_behavior_tree",
        output="screen",
        emulate_tty=True,
        parameters=[configured_params, opponent_pos]
    )

    already_started_nodes = set()

    def start_next_node(event: ProcessStarted, context: LaunchContext):
        print(f'node {event.process_name} started.')
        already_started_nodes.update([event.process_name])
        if len(already_started_nodes) == 2:
            print(f'All required nodes are up, start match BT node')
            return match_node

    return LaunchDescription([

        DeclareLaunchArgument(
            'beacon_present',
            default_value='false',
            description='Does the opponent have beacons ?'),


        DeclareLaunchArgument(
            'opponent_pos',
            default_value='false',
            description='Opponent start position ?'),

        RegisterEventHandler(event_handler=OnProcessStart(target_action=nav_utils_node,
                                                          on_start=start_next_node)),
        RegisterEventHandler(event_handler=OnProcessStart(target_action=table_controller_node,
                                                          on_start=start_next_node)),
        nav_utils_node,
        table_controller_node
    ])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
