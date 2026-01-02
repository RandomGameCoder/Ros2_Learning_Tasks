from launch import LaunchDescription
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
    bridge_config_file = PathJoinSubstitution([sim_pkg, 'config', 'simulation_bridge.yaml'])

    viz_launch = IncludeLaunchDescription(
        viz_launch_file,
        launch_arguments={
            'use_sim_time': 'true',
            'use_joint_state_publisher': 'false',
            'rviz_config': 'visualization'
        }.items()
    )
    sim_launch = IncludeLaunchDescription(
        sim_launch_file,
        launch_arguments={
            'world': 'empty',
            'bridge_params': bridge_config_file,
            'use_description': 'false'
        }.items()
    )

    return LaunchDescription([
        viz_launch,
        sim_launch
    ])