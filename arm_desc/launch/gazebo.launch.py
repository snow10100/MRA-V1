from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package paths
    pkg_arm_desc = get_package_share_directory('arm_desc')
    
    # URDF file path
    urdf_file = os.path.join(pkg_arm_desc, 'urdf', 'robot.urdf')
    
    # World file path
    world_file = '/workspace/worlds/arm_workspace.world'
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-v', '4'],
        output='screen'
    )
    
    # Spawn robot in Gazebo (delayed to ensure Gazebo is ready)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'robotic_arm',
                    '-z', '1.0'
                ],
                output='screen'
            )
        ]
    )
    
    # Bridge between ROS and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge
    ])