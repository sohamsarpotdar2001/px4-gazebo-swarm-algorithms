from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    algo = LaunchConfiguration('algo').perform(context)
    drone_count = int(LaunchConfiguration('drone_count').perform(context))
    logger_delay = float(LaunchConfiguration('logger_delay').perform(context))

    swarm_node_group = GroupAction([
        Node(
            package='drone_swarm',
            executable='waypoint_mission',
            name='waypoint_mission',
            output='screen',
        ),
        Node(
            package='drone_swarm',
            executable=algo,
            name=algo,
            output='screen',
            parameters=[{'drone_count': drone_count}],
        ),
        TimerAction(
            period=logger_delay,
            actions=[
                Node(
                    package='drone_swarm',
                    executable='swarm_logger',
                    name='swarm_logger',
                    output='screen',
                    arguments=[str(drone_count), algo],
                    parameters=[{'drone_count': drone_count}],
                )
            ],
        ),
    ])

    return [swarm_node_group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_count',
            default_value='3',
            description='Number of drones in the swarm',
        ),
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
