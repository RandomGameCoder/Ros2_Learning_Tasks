from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    # Package paths
    desc_pkg = FindPackageShare('tortoisebot_description')
    sim_pkg = FindPackageShare('tortoisebot_gazebo')
    plugins_pkg = FindPackageShare('tortoisebot_gazebo_plugins')
    
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
    plugin_path = PathJoinSubstitution([plugins_pkg, '..', '..', 'lib'])

    bridge_params_lnch_arg = DeclareLaunchArgument(
        'bridge_params',
        default_value=bridge_config_file,
        description='YAML file with ROS-Gazebo bridge configuration'
    )

    # Set environment variable
    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_path
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
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tortoisebot',
            '-topic', '/robot_description',
            '-z', '0.5',
        ]
    )

    return LaunchDescription([
        world_lnch_arg,
        bridge_params_lnch_arg,
        use_description_lnch_arg,
        set_plugin_path,
        desc_launch,
        bridge,
        gz_launch,
        spawn_robot,
    ])