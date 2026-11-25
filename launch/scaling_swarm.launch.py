from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    algo = LaunchConfiguration('algo').perform(context)
    logger_delay = LaunchConfiguration('logger_delay')

    swarm_node_group = GroupAction([
        Node(
            package='drone_swarm',
            executable='mission_pub',
            name='mission_pub',
            output='screen',
        ),

        Node(
            package='drone_swarm',
            executable='mission_reset_bridge',
            name='mission_reset_bridge',
            output='screen',
        ),

        TimerAction(
            period=logger_delay,
            actions=[
                Node(
                    package='drone_swarm',
                    executable='mission_logger',
                    name='mission_logger',
                    output='screen',
                    arguments=[algo],
                )
            ],
        ),
    ])

    return [swarm_node_group]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'algo',
            default_value='consensus',
            description='Algorithm executable and label (consensus, boids, pso, aco, abc, apf)',
        ),
        DeclareLaunchArgument(
            'logger_delay',
            default_value='2.0',
            description='Delay in seconds before starting swarm_logger',
        ),

        OpaqueFunction(function=launch_setup),
    ])
