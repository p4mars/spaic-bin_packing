"""Launch the SpatialAI Gazebo world."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_world = PathJoinSubstitution([
        FindPackageShare('spatial_ai_simulation'),
        'worlds',
        'spatial_ai_station.world',
    ])
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py',
    ])
    spatial_ai_models = PathJoinSubstitution([
        FindPackageShare('spatial_ai_simulation'),
        'gazebo_models',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world file to load.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to false to run Gazebo headless.',
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Set to true for verbose Gazebo output.',
        ),
        DeclareLaunchArgument(
            'pause',
            default_value='false',
            description='Set to true to start Gazebo paused.',
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
                ':',
                spatial_ai_models,
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'),
                'verbose': LaunchConfiguration('verbose'),
                'pause': LaunchConfiguration('pause'),
            }.items(),
        ),
    ])
