from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='config/params.yaml',
            description='Setting voxel grid leaf size'
        ),
        Node(
            package='ros2_pcd_tutorial',
            executable='voxel_grid_processor',
            name='voxel_grid_processor',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='ros2_pcd_tutorial',
            executable='pcd_publisher',
            name='pcd_publisher',
        ),
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', './rviz/visualized_voxel_grid.rviz'])
    ])
