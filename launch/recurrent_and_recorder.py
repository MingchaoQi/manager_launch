import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def get_latest_directory(base_dir):
    subdirs = [d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]
    subdirs.sort(reverse=True)
    if not subdirs:
        raise ValueError("没有找到子目录")
    latest_dir = subdirs[0]
    latest_path = os.path.join(base_dir, latest_dir)
    return latest_path, latest_dir

def generate_launch_description():
    base_dir = '/root/ros2_ws/ros2_driver_layer/training_data/human_teaching'
    latest_path, latest_dir_name = get_latest_directory(base_dir)

    #recorder_vector_node = Node(
    #    package='data_recorder',
    #    executable='multi_topic_recorder_vector',
    #    name='multi_topic_recorder_vector',
    #    output='screen'
    #)

    #recorder_image_node = Node(
    #    package='data_recorder',
    #    executable='multi_topic_recorder_image',
    #    name='multi_topic_recorder_image',
    #    output='screen'
    #)
    
    recorder_node = Node(
        package='data_recorder',
        executable='dataset_recorder',
        name='dataset_recorder',
        output='screen'
    )
    

    bag_play_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', latest_dir_name, '--remap',
            '/topic_recorder:=/lbr/command/joint_position'
        ],
        cwd=base_dir,
        shell=True
    )

    stop_recorder = ExecuteProcess(
        cmd=['pkill', '-f', 'recorder_node'],
        shell=True
    )

    #stop_recorder_image = ExecuteProcess(
    #    cmd=['pkill', '-f', 'multi_topic_recorder_image'],
    #    shell=True
    #)

    return LaunchDescription([
        recorder_node,
        bag_play_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_play_process,
                on_exit=[stop_recorder]
            )
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
