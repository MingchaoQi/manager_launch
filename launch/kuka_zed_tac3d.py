import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_prefix = get_package_share_directory('Tac3D-v3.1.3-linux')
    params_file = os.path.join(pkg_prefix, 'Tac3D-v3.1.3-linux')

    return LaunchDescription([
        # 机械臂lbr_bringup启动
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'lbr_bringup', 'bringup.launch.py',
                'sim:=false',
                'ctrl:=lbr_joint_position_command_controller',
                'model:=iiwa14'
            ],
            shell=True
        ),
        # zed相机启动
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py',
                'camera_model:=zed2i'
            ],
            shell=True
        ),
        # Tac3D触觉传感器执行文件启动
        ExecuteProcess(
            cmd=[
                './Tac3D', '-c', 'config/A1-0040R', '-d', '2', '-i', '127.0.0.1', '-p', '9988'
            ],
            cwd=params_file,
            shell=True,
            output='screen'
        ),
        # tac3d数据publisher节点启动
        ExecuteProcess(
            cmd=['ros2', 'run', 'tac3d', 'tac3d_r'],
            shell=True,
            output='screen'
        )
    ])
