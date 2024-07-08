import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from datetime import datetime


def get_latest_directory(base_dir):
    # 列出基目录中的所有子目录
    subdirs = [d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]

    # 按名称（时间戳）对目录进行排序
    subdirs.sort(reverse=True)

    if not subdirs:
        raise ValueError("没有找到子目录")

    # 最新的目录是排序列表中的第一个
    latest_dir = subdirs[0]
    latest_path = os.path.join(base_dir, latest_dir)

    return latest_path, latest_dir


def generate_launch_description():
    base_dir = '/root/ros2_ws/ros2_driver_layer/training_data/human_teaching'
    latest_path, latest_dir_name = get_latest_directory(base_dir)

    return LaunchDescription([
        Node(
            package='data_recorder',
            executable='multi_topic_recorder_vector',
            name='multi_topic_recorder_vector'
        ),
        Node(
            package='data_recorder',
            executable='multi_topic_recorder_image',
            name='multi_topic_recorder_image'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', latest_dir_name, '--remap',
                '/topic_recorder:=/lbr/command/joint_position'
            ],
            cwd=base_dir,
            shell=True
        )
    ])
