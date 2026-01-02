from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Package paths
    sim_pkg = FindPackageShare('tortoisebot_gazebo')
    viz_pkg = FindPackageShare('tortoisebot_rviz')

    # Launch file paths
    sim_launch_file = PathJoinSubstitution([sim_pkg, 'launch', 'simulation.launch.py'])
    viz_launch_file = PathJoinSubstitution([viz_pkg, 'launch', 'visualization.launch.py'])

    # config file paths
    bridge_config_file = PathJoinSubstitution([sim_pkg, 'config', 'lidar_bridge.yaml'])

    viz_launch = IncludeLaunchDescription(
        viz_launch_file,
        launch_arguments={
            'use_sim_time': 'true',
            'use_joint_state_publisher': 'false',
            'rviz_config': 'lidar_visualization'
        }.items()
    )
    sim_launch = IncludeLaunchDescription(
        sim_launch_file,
        launch_arguments={
            'world': 'lidar_test_world',
            'bridge_params': bridge_config_file,
            'use_description': 'false'
        }.items()
    )
    closest_object_publisher_node = Node(
        package='tortoisebot_sensor_filters',
        executable='closest_object_publisher',
        name='closest_object_publisher',
        output='screen',
    )

    return LaunchDescription([
        viz_launch,
        sim_launch,
        closest_object_publisher_node
    ])