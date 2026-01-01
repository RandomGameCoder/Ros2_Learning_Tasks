from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    use_sim_time_lnch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Find packages
    description_pkg = FindPackageShare('tortoisebot_description')

    # File paths
    urdf_file = PathJoinSubstitution([description_pkg, 'urdf', 'tortoisebot.urdf.xacro'])

    # File contents
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # Launch robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        use_sim_time_lnch_arg,
        robot_state_publisher,
    ])