from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, LogInfo

def generate_launch_description():

    # Load the robot configuration
    moveit_config = MoveItConfigsBuilder("lara8", package_name="moveit_lara8_moveit_config").to_moveit_configs()
    launch_package_path = moveit_config.package_path
    
    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    # Declare the serial number that can be passed from the command line, defaulting to 0
    robot_serial_number = DeclareLaunchArgument(
        'robot_serial_number', default_value='229546', description='Robot serial number'
    )
    ld.add_action(robot_serial_number)  # Add the launch argument to the launch description
    # Log the parameter value to ensure it's being passed correctly
    log_param_value = LogInfo(
        condition=None,
        msg=[LaunchConfiguration('robot_serial_number')]  # Print the value of 'my_param'
    )
    ld.add_action(log_param_value)  # Add the log to print the value

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/lara8_bringup.launch.py")
            ),
            launch_arguments={
            'robot_serial_number': LaunchConfiguration('robot_serial_number')
            }.items(),
        )
    )

    return ld