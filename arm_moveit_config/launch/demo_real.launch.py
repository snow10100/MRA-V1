from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    # MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder("robotic_arm", package_name="arm_moveit_config")
        .robot_description(file_path="config/robotic_arm_real.urdf.xacro")
        .to_moveit_configs()
    )

    # Get parameters for the demo launch
    demo_launch = generate_demo_launch(moveit_config)
    
    return demo_launch.entities


def generate_launch_description():
    declared_arguments = []
    
    # Declare launch arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "device",
            default_value="/dev/ttyACM0",
            description="Serial device for Arduino communication",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "baudrate", 
            default_value="9600",
            description="Baudrate for serial communication"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])