from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler


def generate_launch_description():
    # 创建 open_door_1 节点
    open_door_1 = Node(
        package="tac3d",
        executable="open_door_1",
        name="open_door_1",
    )

    # 创建调用服务的动作
    call_service_1 = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/moveto",
            "bye140_msg/srv/MoveTo",
            '{"position": 25.0, "speed": 200, "acceleration": 1, "torque": 1, "tolerance": 0.01, "waitflag": true}',
        ],
        output="screen",
    )

    # 创建 open_door_2 节点
    open_door_2 = Node(
        package="tac3d",
        executable="open_door_2",
        name="open_door_2",
        condition=None,
    )

    # 创建调用服务的动作
    call_service_2 = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/moveto",
            "bye140_msg/srv/MoveTo",
            '{"position": 95.0, "speed": 200, "acceleration": 1, "torque": 1, "tolerance": 0.01, "waitflag": true}',
        ],
        output="screen",
    )

    # 创建节点 open_door_3
    open_door_3 = Node(
        package="tac3d",
        executable="open_door_3",
        name="open_door_3",
        condition=None,
    )

    # 创建调用服务的动作
    call_service_3 = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/moveto",
            "bye140_msg/srv/MoveTo",
            '{"position": 60.0, "speed": 200, "acceleration": 1, "torque": 1, "tolerance": 0.01, "waitflag": true}',
        ],
        output="screen",
    )

    # 创建节点 open_door_4
    open_door_4 = Node(
        package="tac3d",
        executable="open_door_4",
        name="open_door_4",
        condition=None,
    )

    # 创建调用服务的动作
    call_service_4 = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/moveto",
            "bye140_msg/srv/MoveTo",
            '{"position": 100.0, "speed": 2000, "acceleration": 2000, "torque": 1, "tolerance": 0.01, "waitflag": true}',
        ],
        output="screen",
    )

    # 监听 open_door_1 完成的事件，并启动调用服务1
    event_handler_1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=open_door_1,
            on_exit=[
                LogInfo(msg="open_door_1 exit. Path Planning node finished. Calling MoveTo service."),
                call_service_1,
            ],
        )
    )

    # 监听服务调用完成，并启动 open_door_2
    event_handler_2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=call_service_1,
            on_exit=[
                LogInfo(msg="call_service_1 exit. MoveTo service call finished. Launching Open Door node."),
                open_door_2,
            ],
        )
    )

    # 监听 open_door_2 完成的事件，并启动调用服务2
    event_handler_3 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=open_door_2,
            on_exit=[
                LogInfo(msg="open_door_2 exit. Path Planning node finished. Calling MoveTo service."),
                call_service_2,
            ],
        )
    )

    # 监听服务2调用完成，并启动 open_door_3
    event_handler_4 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=call_service_2,
            on_exit=[
                LogInfo(msg="call_service_2 exit. MoveTo service call finished. Launching Open Door node."),
                open_door_3,
            ],
        )
    )

    # 监听 open_door_3 完成的事件，并启动调用服务3
    event_handler_5 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=open_door_3,
            on_exit=[
                LogInfo(msg="open_door_3 exit. Path Planning node finished. Calling MoveTo service."),
                call_service_3,
            ],
        )
    )

    # 监听服务3调用完成，并启动 open_door_4
    event_handler_6 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=call_service_3,
            on_exit=[
                LogInfo(msg="call_service_3 exit. MoveTo service call finished. Launching Open Door node."),
                open_door_4,
            ],
        )
    )

    # 监听 open_door_3 完成的事件，并启动调用服务3
    event_handler_7 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=open_door_4,
            on_exit=[
                LogInfo(msg="open_door_4 exit. Path Planning node finished. Calling MoveTo service."),
                call_service_4,
            ],
        )
    )

    return LaunchDescription(
        [
            open_door_1,
            event_handler_1,
            event_handler_2,
            event_handler_3,
            event_handler_4,
            event_handler_5,
            event_handler_6,
            event_handler_7,
        ]
    )
