from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler


def generate_launch_description():
    # 创建 open_door_1 节点
    open_drawer_1 = Node(
        package="tac3d",
        executable="open_drawer_1",
        name="open_drawer_1",
    )

    # 创建 open_door_2 节点
    open_drawer_3 = Node(
        package="tac3d",
        executable="open_drawer_3",
        name="open_drawer_3",
        condition=None,
    )

    # 创建调用服务的动作
    call_service_1 = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/moveto",
            "bye140_msg/srv/MoveTo",
            '{"position": 55, "speed": 200, "acceleration": 1, "torque": 1, "tolerance": 0.01, "waitflag": true}',
        ],
        output="screen",
    )

    # 创建节点 open_door_3
    open_drawer_4 = Node(
        package="tac3d",
        executable="open_drawer_4",
        name="open_drawer_4",
        condition=None,
    )

    # 监听 open_door_1 完成的事件，并启动调用服务1
    event_handler_1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=open_drawer_1,
            on_exit=[
                LogInfo(msg="open_drawer_1 exit. Path Planning node finished. Calling MoveTo service."),
                open_drawer_3,
            ],
        )
    )

    # 监听服务调用完成，并启动 open_door_2
    event_handler_2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=open_drawer_3,
            on_exit=[
                LogInfo(msg="open_drawer_3 exit. MoveTo service call finished. Launching Open Door node."),
                call_service_1,
            ],
        )
    )

    # 监听 open_door_2 完成的事件，并启动调用服务2
    event_handler_3 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=call_service_1,
            on_exit=[
                LogInfo(msg="call_service_1 exit. Path Planning node finished. Calling MoveTo service."),
                open_drawer_4,
            ],
        )
    )

    return LaunchDescription(
        [
            open_drawer_1,
            event_handler_1,
            event_handler_2,
            event_handler_3,
        ]
    )
