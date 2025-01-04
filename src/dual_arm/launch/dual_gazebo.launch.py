import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare

def get_robot_description(context: LaunchContext):   
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        'dual_arm',
        'dual_arm' + '.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        franka_xacro_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': True},
        ]
    )
    return [robot_state_publisher]


def prepare_launch_description():
    # Configure ROS nodes for launch
    bridge_params = os.path.join(
        get_package_share_directory('dual_arm'),
        'config',
        'dual_bridge.yaml'
    )
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    # Get robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description)
    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )
    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )
    fr3_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'fr3_arm_controller'],
        output='screen'
    )
    fp3_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'fp3_arm_controller'],
        output='screen'
    )
    fr3_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'fr3_hand_controller'],
        output='screen'
    )
    fp3_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'fp3_hand_controller'],
        output='screen'
    )
    return LaunchDescription([
        gazebo_empty_world,
        robot_state_publisher,
        spawn,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),    
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[fr3_arm_controller,fp3_arm_controller,fr3_gripper_controller,fp3_gripper_controller],
            )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30,
                 'use_sim_time': True
                 }
                 ],
        ),
        start_gazebo_ros_bridge_cmd,
    ])

def generate_launch_description():
    launch_description = prepare_launch_description()
    package_path = get_package_share_directory('franka_description')
    #print("Franka description package path:", package_path)
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(package_path))
    launch_description.add_action(set_env_vars_resources)
    return launch_description
