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
def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file, 
        mappings={
            'arm_id': arm_id_str, 
            'hand': load_gripper_str, 
            'ros2_control': 'true', 
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true',
            'robot_ip':'dont-care ',
            'use_fake_hradware':'true',
            'fakse_sensor_commands': 'false'
        }
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
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
            load_gripper_name,
            default_value='true',
            description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
            franka_hand_name,
            default_value='franka_hand',
            description='Default value: franka_hand')
    arm_id_launch_argument = DeclareLaunchArgument(
            arm_id_name,
            default_value='fr3',
            description='Available values: fr3, fp3 and fer')

    bridge_params = os.path.join(
        get_package_share_directory('panda_gazebo'),
        'config',
        'bridge.yaml'
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
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand])

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '/home/xiaomeng/code/robot_ws/src/robot_gazebo/panda_gazebo/worlds/worlds.sdf -r', }.items(),
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
 
    fr3_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'fr3_gripper_controller'],
        output='screen'
    )


#     start_gazebo_ros_image_bridge_cmd = Node(
#     package='ros_gz_image',
#     executable='image_bridge',
#     arguments=['/camera/image_raw'],
#     parameters=[{'use_sim_time': True}],
#     output='screen',
#    )
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                    '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                    '/top_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/top_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/top_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                    '/top_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                    ],
                     parameters=[{'use_sim_time': True}],
        output='screen'
    )


    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
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
                on_exit=[fr3_arm_controller,fr3_gripper_controller],
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
        # start_gazebo_ros_bridge_cmd,
        #start_gazebo_ros_image_bridge_cmd,
        bridge,
        # 启动 pointcloud_frame_modifier.py 节点
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
