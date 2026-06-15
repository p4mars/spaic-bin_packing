import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('mirte_perception')

    # Default model paths — override with:
    #   ros2 launch mirte_perception perception.launch.py model1_path:=/abs/path/model.pt
    default_models_dir = os.path.join(
        os.path.expanduser('~'),
        'spatial-ai', 'ws', 'src', 'mirte-ros-packages',
        'mirte_perception', 'models',
    )

    model1_arg = DeclareLaunchArgument(
        'model1_path',
        default_value=os.path.join(default_models_dir, 'handles_model.pt'),
        description='Absolute path to first YOLOv8 .pt weights file',
    )
    conf_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.7',
        description='Minimum confidence score to keep a detection',
    )
    frame_arg = DeclareLaunchArgument(
        'fixed_frame',
        default_value='camera_depth_optical_frame',
        description='TF frame used for 3D marker positions',
    )

    perception_node = Node(
        package='mirte_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{
            'model1_path': LaunchConfiguration('model1_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'fixed_frame': LaunchConfiguration('fixed_frame'),
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        model1_arg,
        conf_arg,
        frame_arg,
        perception_node,
    ])
