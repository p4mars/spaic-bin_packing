import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_models_dir = os.path.join(
        os.path.expanduser('~'),
        'spatial-ai', 'ws', 'src', 'mirte-ros-packages',
        'mirte_perception', 'models',
    )

    return LaunchDescription([
        DeclareLaunchArgument('model1_path',
            default_value=os.path.join(default_models_dir, 'handles_model.pt'),
            description='Path to YOLOv8 handles weights'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.7'),
        DeclareLaunchArgument('handle_real_width_m', default_value='0.02'),
        DeclareLaunchArgument('gripper_model_path',
            default_value=os.path.join(default_models_dir, 'gripper_model.pt'),
            description='Path to YOLOv8 weights for top-down gripper camera detection'),

        Node(
            package='mirte_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'model1_path': LaunchConfiguration('model1_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'handle_real_width_m': LaunchConfiguration('handle_real_width_m'),
                'fixed_frame': 'camera_depth_optical_frame',
            }],
        ),

        Node(
            package='mirte_perception',
            executable='grasp_node',
            name='grasp_node',
            output='screen',
            parameters=[{
                'gripper_model_path': LaunchConfiguration('gripper_model_path'),
            }],
        ),
    ])
