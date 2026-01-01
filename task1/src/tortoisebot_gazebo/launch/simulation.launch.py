from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Package paths
    desc_pkg = FindPackageShare('tortoisebot_description')
    sim_pkg = FindPackageShare('tortoisebot_gazebo')
    bridge_pkg = FindPackageShare('ros_gz_bridge')
    
    # Launch configuration
    world = LaunchConfiguration('world')
    bridge_params = LaunchConfiguration('bridge_params')

    # Declare launch arguments
    world_lnch_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World to load in gazebo'
    )

    # File paths
    bridge_launch_file = PathJoinSubstitution([bridge_pkg, 'launch', 'ros_gz_bridge.launch.py'])
    world_file = PathJoinSubstitution([sim_pkg, 'sdf', PythonExpression(["'", world, "' + '.sdf'"])])
    urdf_file = PathJoinSubstitution([desc_pkg, 'urdf', 'tortoisebot.urdf'])
    bridge_config_file = PathJoinSubstitution([sim_pkg, 'config', "simulation_bridge.yaml"])

    bridge_params_lnch_arg = DeclareLaunchArgument(
        'bridge_params',
        default_value=bridge_config_file,
        description='YAML file with ROS-Gazebo bridge configuration'
    )

    # Launch descriptions
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        parameters=[{
            'config_file': bridge_params
        }],
    )
    gz_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tortoisebot',
            '-file', urdf_file,
            '-z', '0.5',
        ]
    )

    return LaunchDescription([
        world_lnch_arg,
        bridge_params_lnch_arg,
        bridge,
        gz_launch,
        spawn_robot,
    ])