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
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import RegisterEventHandler,ExecuteProcess
from launch.event_handlers import OnProcessExit


 
def generate_launch_description():

    declared_arguments = []

    moveit_config = (
        MoveItConfigsBuilder("moveit_lara8")
        .robot_description(file_path="config/lara8_gaz.urdf.xacro")
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
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )
   

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",   
            default_value=PathJoinSubstitution(
                [FindPackageShare("moveit_lara8_moveit_config"), "config/moveit.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",

        parameters=[moveit_config.robot_description,
              {"use_sim_time": True}     
            ],

    )

    # Load controllers
    load_controllers = []
    for controller in [
       
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



    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        parameters=[ moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}],
        
    )

    world_path=os.path.join(get_package_share_directory('moveit_lara8_moveit_config'),'gazebo/worlds','myworld.sdf')

    declared_arguments.append(DeclareLaunchArgument('gz_args', default_value=f'-r -v 1 {world_path}',
                              description='Arguments for gz_sim'),)

    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   'controller_manager'],
    )

    right_lara8_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_lara8_controller', '--controller-manager',
                   'controller_manager'],
    )

    left_lara8_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_lara8_controller', '--controller-manager',
                   'controller_manager'],
    )


    

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'lara8',
                   '-allow_renaming', 'true',
                    ],
    )
 
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
           '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '--ros-args', 
            '-r', '/camera:=/videocamera',
        ],
        output='screen'
    ) 


    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        ),
    )


    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
       
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[right_lara8_controller_spawner,left_lara8_controller_spawner],
        )
    )

   
    nodes_to_start = [
             
        robot_state_publisher_node,      
        gazebo_ignition,
        gz_spawn_entity,
        bridge_camera,
        move_group_node,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner       
    ]



    return LaunchDescription( declared_arguments+nodes_to_start) 
