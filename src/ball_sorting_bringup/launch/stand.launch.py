"""Стенд: симуляция, контроллеры и вспомогательные узлы задачи.

Запускает сцену MuJoCo с ros2_control и контроллерами (simulation.launch.py), имитацию
технического зрения /targets_detector и подачу шаров /targets_launcher. Узлов системы
управления манипулятором здесь нет: их разрабатывают по ходу курса.

Запуск:
    ros2 launch ball_sorting_bringup stand.launch.py
    ros2 launch ball_sorting_bringup stand.launch.py headless:=true

После запуска:
    ros2 service call /targets_launcher/launch std_srvs/srv/SetBool "{data: true}"  # включить подачу шаров
    ros2 topic echo /targets_detector/detected_targets                              # шары, пригодные для захвата
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("ball_sorting_bringup")
    config = lambda name: ParameterFile(PathJoinSubstitution([package_share, "config", name]))  # noqa: E731
    sim_time = {"use_sim_time": True}

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([package_share, "launch", "simulation.launch.py"])),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "sim_speed_factor": LaunchConfiguration("sim_speed_factor"),
        }.items(),
    )
    targets_detector = Node(
        package="targets_detector",
        executable="targets_detector",
        parameters=[config("targets_detector.yaml"), sim_time],
        output="screen",
    )
    targets_launcher = Node(
        package="targets_launcher",
        executable="targets_launcher",
        parameters=[config("targets_launcher.yaml"), sim_time],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Запустить MuJoCo без окна (нужно, если нет графического сеанса X11).",
            ),
            DeclareLaunchArgument(
                "sim_speed_factor",
                default_value="1.0",
                description="Скорость симуляции относительно реального времени.",
            ),
            simulation,
            targets_detector,
            targets_launcher,
        ]
    )
