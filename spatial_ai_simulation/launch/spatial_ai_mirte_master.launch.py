"""Launch the SpatialAI Gazebo world with a MIRTE Master robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare('spatial_ai_simulation'),
        'worlds',
        'spatial_ai_station.world',
    ])
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py',
    ])
    spawn_mirte_launch = PathJoinSubstitution([
        FindPackageShare('mirte_gazebo'),
        'launch',
        'spawn_mirte_master.launch.xml',
    ])
    spatial_ai_models = PathJoinSubstitution([
        FindPackageShare('spatial_ai_simulation'),
        'gazebo_models',
    ])
    mirte_models = PathJoinSubstitution([
        FindPackageShare('mirte_gazebo'),
        'models',
    ])
    mirte_media = PathJoinSubstitution([
        FindPackageShare('mirte_gazebo'),
        'media',
    ])
    twist_mux_config = PathJoinSubstitution([
        FindPackageShare('mirte_gazebo'),
        'config',
        'twist_mux.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('pause', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('arm_enable', default_value='True'),
        DeclareLaunchArgument('sonar_enable', default_value='True'),
        DeclareLaunchArgument('lidar_enable', default_value='True'),
        DeclareLaunchArgument('depth_camera_enable', default_value='True'),
        DeclareLaunchArgument('use_twist_mux', default_value='true'),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
                ':',
                spatial_ai_models,
                ':',
                mirte_models,
            ],
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MEDIA_PATH',
            value=[
                EnvironmentVariable('GAZEBO_MEDIA_PATH', default_value=''),
                ':',
                mirte_media,
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world,
                'gui': LaunchConfiguration('gui'),
                'verbose': LaunchConfiguration('verbose'),
                'pause': LaunchConfiguration('pause'),
            }.items(),
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(spawn_mirte_launch),
            launch_arguments={
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'roll': LaunchConfiguration('roll'),
                'pitch': LaunchConfiguration('pitch'),
                'yaw': LaunchConfiguration('yaw'),
                'arm_enable': LaunchConfiguration('arm_enable'),
                'sonar_enable': LaunchConfiguration('sonar_enable'),
                'lidar_enable': LaunchConfiguration('lidar_enable'),
                'depth_camera_enable': LaunchConfiguration('depth_camera_enable'),
            }.items(),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                'mirte_master_arm_controller',
                'mirte_master_gripper_controller',
            ],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['pid_wheels_controller', 'mirte_base_controller'],
            parameters=[{'use_sim_time': True}],
        ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('use_twist_mux')),
            cmd=[
                'ros2',
                'topic',
                'pub',
                '/zero_cmd_vel',
                'geometry_msgs/msg/Twist',
                '{}',
                '-r',
                '100',
            ],
            output='log',
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_twist_mux')),
            package='twist_mux',
            executable='twist_mux',
            remappings=[('/cmd_vel_out', '/cmd_vel')],
            parameters=[twist_mux_config, {'use_sim_time': True}],
        ),
    ])
