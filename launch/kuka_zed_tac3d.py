import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    # 机械臂lbr_bringup启动
    lbr_bringup = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'lbr_bringup', 'bringup.launch.py',
            'sim:=false',
            'ctrl:=lbr_joint_position_command_controller',
            'model:=iiwa14'
        ],
        shell=True
    )
    ld.add_action(lbr_bringup)

    # Tac3D触觉传感器执行文件启动
    tac3d = ExecuteProcess(
        cmd=[
            './Tac3D', '-c', 'config/A1-0040R', '-d', '2', '-i', '127.0.0.1', '-p', '9988'
        ],
        cwd='/root/ros2_ws/ros2_driver_layer/src/Tac3D-v3.1.3-linux',
        shell=True,
        output='screen'
    )
    ld.add_action(tac3d)

    # tac3d数据publisher节点启动
    tac3d_publisher = ExecuteProcess(
        cmd=['ros2', 'run', 'tac3d', 'tac3d_r'],
        shell=True,
        output='screen'
    )
    ld.add_action(tac3d_publisher)

    # 进程退出事件处理器
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=lbr_bringup,
            on_exit=[tac3d, tac3d_publisher]
        )
    ))

    return ld


