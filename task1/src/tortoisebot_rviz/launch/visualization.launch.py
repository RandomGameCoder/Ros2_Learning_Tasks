from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    rviz_config = LaunchConfiguration('rviz_config')

    # Declare launch arguments
    use_sim_time_lnch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    use_joint_state_publisher_lnch_arg = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='true',
        description='Whether to launch the joint state publisher'
    )
    rviz_config_lnch_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='visualization',
        choices=['visualization'],
        description='RViz config type to use'
    )

    # Find packages
    description_pkg = FindPackageShare('tortoisebot_description')
    rviz_pkg = FindPackageShare('tortoisebot_rviz')

    # File paths
    urdf_file = PathJoinSubstitution([description_pkg, 'urdf', 'tortoisebot.urdf'])
    rviz_config_file = PathJoinSubstitution([rviz_pkg, 'config', PythonExpression(["'", rviz_config, "' + '.rviz'"])])
    # File contents
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # Launch robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Launch Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_joint_state_publisher)
    )

    # Launch RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='visualization_rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_lnch_arg,
        use_joint_state_publisher_lnch_arg,
        rviz_config_lnch_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])