import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    drone_count_str = LaunchConfiguration('drone_count').perform(context)
    drone_count = int(drone_count_str)

    px4_dir = os.path.expanduser(
        "~/PX4-Autopilot"
    )

    px4_binary = os.path.join(
        px4_dir,
        "build",
        "px4_sitl_default",
        "bin",
        "px4",
    )

    drones = []
    for i in range(drone_count):
        instance = i + 1
        x = ((-1)**i) * i
        y = ((-1)**i) * i
        drones.append((instance, str(x), str(y)))
    
    processes = []

    for index, (instance, x, y) in enumerate(drones):

        environment = os.environ.copy()

        environment["PX4_SYS_AUTOSTART"] = "4001"
        environment["PX4_GZ_NO_FOLLOW"] = "1"
        environment["PX4_SIM_MODEL"] = "gz_x500"
        environment["PX4_DRONE_COUNT"] = str(drone_count)
        environment["PX4_GZ_MODEL_POSE"] = f"{x},{y}"
        environment["PX4_GZ_WORLD"] = "default"
        
        # Only the first PX4 instance starts Gazebo.
        if index > 0:
            environment["PX4_GZ_STANDALONE"] = "1"

        px4_process = ExecuteProcess(
            cmd=[px4_binary, "-i", str(instance)],
            cwd=px4_dir,
            env=environment,
            output="screen",
            shell=False,
        )

        if index == 0:
            processes.append(px4_process)
        else:
            processes.append(
                TimerAction(
                    period=5.0 * index,
                    actions=[px4_process],
                )
            )

    processes.append(
        ExecuteProcess(
            cmd=[
                "MicroXRCEAgent",
                "udp4",
                "-p",
                "8888",
            ],
            output="screen",
            shell=False,
        )
    )

    return processes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_count',
            default_value='3',
            description='Number of drones in Gazebo',
        ),

        OpaqueFunction(function=launch_setup),
    ])
