from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments

    # Package paths
    desc_pkg = FindPackageShare('tortoisebot_description')
    sim_pkg = FindPackageShare('tortoisebot_gazebo')
    bridge_pkg = FindPackageShare('ros_gz_bridge')

    
    # Launch configuration
    world = LaunchConfiguration('world')
    use_description = LaunchConfiguration('use_description', default='true')
    bridge_params = LaunchConfiguration('bridge_params')

    # Declare launch arguments
    world_lnch_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World to load in gazebo'
    )
    use_description_lnch_arg = DeclareLaunchArgument(
        'use_description',
        default_value='true',
        description='Whether to load the robot description'
    )

    # File paths
    desc_launch_file = PathJoinSubstitution([desc_pkg, 'launch', 'description.launch.py'])
    world_file = PathJoinSubstitution([sim_pkg, 'sdf', PythonExpression(["'", world, "' + '.sdf'"])])
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
    desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(desc_launch_file),
        condition=IfCondition(use_description)
    )
    gz_launch = ExecuteProcess(
        cmd=['gz', 'sim', world_file, "--physics-engine", "gz-physics-dart-plugin", "--verbose"],
        output='screen'
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tortoisebot',
            # '-file', urdf_file,
            '-topic', '/robot_description',
            '-z', '0.5',
            '-p', '1.5708'
        ]
    )

    return LaunchDescription([
        world_lnch_arg,
        bridge_params_lnch_arg,
        use_description_lnch_arg,
        desc_launch,
        bridge,
        gz_launch,
        spawn_robot,
    ])