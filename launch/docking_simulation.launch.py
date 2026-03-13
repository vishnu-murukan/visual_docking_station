#!/usr/bin/env python3
"""
Launch file for visual docking simulation
ROS2 Jazzy + Gazebo Harmonic
"""
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('visual_docking')

    world_file     = os.path.join(pkg_share, 'worlds', 'room_world.sdf')
    robot_sdf_file = os.path.join(pkg_share, 'models', 'docking_robot', 'model.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo needs to find custom models (aruco_marker + docking_robot)
    gz_models_path  = os.path.join(pkg_share, 'models')
    existing_path   = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    full_model_path = gz_models_path + (':' + existing_path if existing_path else '')
    set_gz_resource = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', full_model_path)

    # ---------- Gazebo ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ---------- Spawn robot from SDF ----------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'docking_robot',
            '-file', robot_sdf_file,
            '-x', '3.0',
            '-y', '0.5',
            '-z', '0.05',     # just above ground; robot will settle
            '-Y', '3.14159',  # facing the dock on the far wall
        ],
        output='screen'
    )

    # ---------- ROS <-> Gazebo bridge ----------
    #
    # DiffDrive plugin uses RELATIVE topic names inside the model namespace:
    #   cmd_vel  -> Gazebo transport: /model/docking_robot/cmd_vel
    #   odom     -> Gazebo transport: /model/docking_robot/odom
    #   tf       -> Gazebo transport: /model/docking_robot/tf
    #
    # The bridge maps these to standard ROS topics via remappings.
    #
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Camera: GZ → ROS
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Odom: GZ → ROS  (scoped by model name)
            '/model/docking_robot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # cmd_vel: ROS → GZ  (scoped by model name)
            '/model/docking_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # TF: GZ → ROS  (scoped by model name)
            '/model/docking_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            # Remap scoped topics to standard ROS names
            ('/model/docking_robot/odom',    '/odom'),
            ('/model/docking_robot/cmd_vel', '/cmd_vel'),
            ('/model/docking_robot/tf',      '/tf'),
        ],
        output='screen'
    )

    # ---------- ArUco detector ----------
    aruco_detector = Node(
        package='visual_docking',
        executable='aruco_detector_node.py',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tag_size':     0.36,
            'marker_id':    0,
            'camera_topic': '/camera/image_raw'
        }]
    )

    # ---------- Dock controller ----------
    dock_controller = Node(
        package='visual_docking',
        executable='dock_controller.py',
        name='dock_controller',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'stop_distance': 0.55
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        set_gz_resource,
        gazebo,
        spawn_robot,
        bridge,
        aruco_detector,
        dock_controller,
    ])
