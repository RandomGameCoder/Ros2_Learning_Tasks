from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
        choices=['visualization', 'lidar_visualization'],
        description='RViz config type to use'
    )
    

    # Find packages
    rviz_pkg = FindPackageShare('tortoisebot_rviz')
    desc_pkg = FindPackageShare('tortoisebot_description')

    # File paths
    desc_launch_file = PathJoinSubstitution([desc_pkg, 'launch', 'description.launch.py'])
    rviz_config_file = PathJoinSubstitution([rviz_pkg, 'config', PythonExpression(["'", rviz_config, "' + '.rviz'"])])

    # Launch robot_state_publisher
    desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(desc_launch_file),
        launch_arguments={'use_sim_time': use_sim_time}.items()
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
        desc_launch,
        joint_state_publisher_gui,
        rviz2,
    ])