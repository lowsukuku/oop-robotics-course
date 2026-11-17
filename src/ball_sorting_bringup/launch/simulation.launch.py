"""Симуляция без узлов задачи: сцена MuJoCo, ros2_control и контроллеры.

Запуск:
    ros2 launch ball_sorting_bringup simulation.launch.py
    ros2 launch ball_sorting_bringup simulation.launch.py headless:=true

Все узлы работают по времени симуляции (use_sim_time): его публикует в /clock физический
поток MuJoCo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

CONTROLLERS = ("joint_state_broadcaster", "arm_controller", "gripper_controller")


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("ball_sorting_bringup")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("ball_sorting_description"), "description", "ur5e_2f85.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution([package_share, "config", "controllers.yaml"])
    plugins_file = PathJoinSubstitution([package_share, "config", "mujoco_plugins.yaml"])

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " headless:=",
                LaunchConfiguration("headless"),
                " sim_speed_factor:=",
                LaunchConfiguration("sim_speed_factor"),
            ]
        ),
        value_type=str,
    )

    # Публикует описание робота в topic /robot_description (его читает controller_manager) и TF звеньев.
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # Один процесс: /controller_manager, контроллеры, /mujoco_ros2_control_node и физика MuJoCo.
    # Завершение процесса (например, закрытие окна MuJoCo) останавливает весь запуск.
    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": True},
            ParameterFile(controllers_file),
            ParameterFile(plugins_file),
        ],
        output="screen",
        on_exit=Shutdown(),
    )

    # Спавнер ждёт /controller_manager, затем загружает контроллер и активирует его.
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager-timeout", "60"],
            output="screen",
        )
        for controller in CONTROLLERS
    ]

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
            state_publisher,
            control_node,
            *spawners,
        ]
    )
