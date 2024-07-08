import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # 获取参数文件的路径
    pkg_prefix = get_package_share_directory('lbr_demos_advanced_py')
    params_file = os.path.join(pkg_prefix, 'config', 'admittance_control.yaml')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'data_recorder', 'topic_recorder'],
            shell=True
        ),
        ExecuteProcess(

            cmd=[
                # ros2 run lbr_demos_advanced_py admittance_control --ros-args \
                #     -r __ns:=/lbr \
                #     --params-file `ros2 pkg prefix lbr_demos_advanced_py`/share/lbr_demos_advanced_py/config/admittance_control.yaml
                'ros2', 'run', 'lbr_demos_advanced_py', 'admittance_control', '--ros-args',
                '-r', '__ns:=/lbr',
                '--params-file', params_file
            ],
            shell=True
        ),
    ])
