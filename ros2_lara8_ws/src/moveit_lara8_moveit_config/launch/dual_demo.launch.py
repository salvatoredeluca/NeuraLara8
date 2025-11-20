import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import  RegisterEventHandler,ExecuteProcess,DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    declared_arguments = []
   
    moveit_config = (
        MoveItConfigsBuilder("moveit_lara8")
        .robot_description(file_path="config/lara8.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_lara8.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)  
        .robot_description_kinematics(file_path="config/kinematics.yaml") 
        .to_moveit_configs()
        
        
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),{'use_sim_time':False}],
    )

    #RViz
    rviz_config = os.path.join(
        get_package_share_directory("moveit_lara8_moveit_config"),
        "config/moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False}
           
           
        ],
    )
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {'use_sim_time': False}],
       
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_lara8_moveit_config"),
        "config/",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )
   

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "left_lara8_controller",
        "right_lara8_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        declared_arguments+
        [       
            robot_state_publisher,
            rviz_node,
            move_group_node,       
            ros2_control_node,
            

        ]
        + load_controllers
    )