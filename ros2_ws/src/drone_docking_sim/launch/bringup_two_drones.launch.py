from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    package_name = "drone_docking_sim"

    # Default world: installed world inside the package share
    default_world = PathJoinSubstitution([
        FindPackageShare(package_name),
        "worlds",
        "two_iris.sdf",
    ])

    world = LaunchConfiguration("world")

    # Where you cloned ardupilot_gazebo (inside the distrobox)
    ardupilot_gz = os.path.expanduser("~/ardupilot_gazebo")

    # Build GZ_SIM_RESOURCE_PATH without trying to "perform" substitutions
    # We include:
    #  - existing env var
    #  - your package share (so Fuel URLs aside, world/model URIs can resolve if you add local assets later)
    #  - ardupilot_gazebo models/worlds
    gz_sim_resource_path = [
        EnvironmentVariable("GZ_SIM_RESOURCE_PATH"),
        TextSubstitution(text=":"),
        FindPackageShare(package_name),
        TextSubstitution(text=":"),
        TextSubstitution(text=os.path.join(ardupilot_gz, "models")),
        TextSubstitution(text=":"),
        TextSubstitution(text=os.path.join(ardupilot_gz, "worlds")),
    ]

    set_gz_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gz_sim_resource_path,
    )

    # Start Gazebo Harmonic
    gz = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen",
    )

    # Two ArduPilot SITL instances (assumes ~/ardupilot exists and sim_vehicle.py works)
    sitl1 = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "cd ~/ardupilot && "
            "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON "
            "--no-mavproxy --no-console -I0"
        ],
        output="screen",
    )

    sitl2 = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "cd ~/ardupilot && "
            "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON "
            "--no-mavproxy --no-console -I1"
        ],
        output="screen",
    )

    # OPTIONAL: Start one MAVProxy instance connected to both vehicles.
    # Comment this out for now if you don't have MAVProxy installed yet.
    mavproxy = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "mavproxy.py "
            "--master=tcp:127.0.0.1:5760 "
            "--master=tcp:127.0.0.1:5770 "
            "--console"
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Path to SDF world file",
        ),
        set_gz_path,
        gz,
        sitl1,
        sitl2,
        mavproxy,
    ])