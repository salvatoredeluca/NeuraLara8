from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
import os
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


 
def generate_launch_description():
    declared_arguments = []

     
    # x=LaunchConfiguration('x')
    # y=LaunchConfiguration('y')
    namespace = LaunchConfiguration('namespace') 
    prefix=LaunchConfiguration('prefix')
   
    
    lara_path = os.path.join(
        get_package_share_directory('robot_description'))
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",   
            default_value="first",
            description="Namespaces to enable multi-robot simulation",
        )
    )
    

    
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",   
            default_value="first",
            description="Namespaces to enable multi-robot simulation",
        )
    )
    
  

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",   
            default_value=PathJoinSubstitution(
                [FindPackageShare("robot_description"), "config.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )

    xacro_arm = os.path.join(lara_path, "lara8/urdf", "lara8.urdf.xacro")

    robot_description_arm_xacro_1 = {"robot_description": Command(['xacro ', xacro_arm," ",'prefix:=',prefix," "
            'namespace:=',
            namespace])}
    
    robot_description_arm_xacro_2 = {"robot_description": Command(['xacro ', xacro_arm," ",'prefix:=second'," "
            'namespace:=',
            namespace])}


    joint_state_publisher_node_1 = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="first",
    )


    joint_state_publisher_node_2 = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace="second",
    )


 
    robot_state_publisher_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace="first",
        parameters=[robot_description_arm_xacro_1,
                    {"use_sim_time": True},
            ],

        
        #remappings=[('/robot_description', '/first/robot_description')]
    )

    robot_state_publisher_node_2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace="second",
        parameters=[robot_description_arm_xacro_2,
                    {"use_sim_time": True},
            ],

        
        #remappings=[('/robot_description', '/first/robot_description')]
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        
    )
    
    nodes_to_start = [
        robot_state_publisher_node_1,
        robot_state_publisher_node_2,
        joint_state_publisher_node_1,
        joint_state_publisher_node_2,
        #joint_state_broadcaster,
        rviz_node
        
    ]


    return LaunchDescription( declared_arguments+nodes_to_start) 
