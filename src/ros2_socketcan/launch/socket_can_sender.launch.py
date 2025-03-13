# socket_can_sender.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    socket_can_sender_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='socket_can_sender',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            # Convert the string argument to boolean using PythonExpression.
            'enable_can_fd': PythonExpression(["'", LaunchConfiguration('enable_can_fd'), "' == 'true'"]),
            'timeout_sec': LaunchConfiguration('timeout_sec'),
        }],
        remappings=[('to_can_bus', LaunchConfiguration('to_can_bus_topic')),
                    ('to_can_bus_fd', LaunchConfiguration('to_can_bus_topic'))],
        output='screen')

    socket_can_sender_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_sender_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    socket_can_sender_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_sender_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can0'),
        DeclareLaunchArgument('enable_can_fd', default_value='false'),
        DeclareLaunchArgument('timeout_sec', default_value='0.01'),
        DeclareLaunchArgument('auto_configure', default_value='true'),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        # Topic remapping: if CAN FD is enabled, use a different topic.
        DeclareLaunchArgument('to_can_bus_topic', default_value='to_can_bus_fd',
                              condition=IfCondition(LaunchConfiguration('enable_can_fd'))),
        DeclareLaunchArgument('to_can_bus_topic', default_value='to_can_bus',
                              condition=UnlessCondition(LaunchConfiguration('enable_can_fd'))),
        socket_can_sender_node,
        socket_can_sender_configure_event_handler,
        socket_can_sender_activate_event_handler,
    ])
