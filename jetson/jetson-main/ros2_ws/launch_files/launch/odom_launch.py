from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ekf_launch_path = os.path.join(
        get_package_share_directory('IMU_odom'),
        'launch',
        'ekf.launch.py'
    )

    return LaunchDescription([
        Node(
            package='odometry',
            executable='odom',
            name='odometry_node',
            output='screen'
        ),
        Node(
            package='imu_data',
            executable='imu',
            name='imu_node',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch_path)
        )
    ])

