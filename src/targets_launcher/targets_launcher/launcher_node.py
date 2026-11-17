"""ROS 2-узел /targets_launcher: вспомогательная подача шаров в рабочую зону.

Узел не входит в ПО робота: он имитирует поступление шаров. По сервису ~/launch
подача включается и выключается. Пока она включена, через случайные интервалы узел
выбирает случайный шар среди лежащих в контейнерах и переносит его в точку подачи
сервисом симулятора set_free_joint_state. Подача пропускается, если в рабочей зоне
уже max_balls_in_zone шаров или больше: иначе зона заполняется шарами, лежащими
вплотную, и пальцы захвата задевают соседние шары.
Положения шаров узел получает из истинного состояния симулятора (topic
free_joint_states) и не зависит от /targets_detector.
"""

from __future__ import annotations

import random

import rclpy
import rclpy.duration
import rclpy.logging
from mujoco_ros2_control_msgs.msg import FreeJointState, FreeJointStateArray
from mujoco_ros2_control_msgs.srv import SetFreeJointState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool

from targets_launcher.launching import (
    BallPosition,
    Bin,
    Zone,
    balls_in_bins,
    balls_in_zone,
    choose_ball,
    direction_from_nearest_wall,
    random_launch_velocity,
)

CHECK_PERIOD = 0.1  # с: как часто узел проверяет, не пора ли подать шар


class TargetsLauncher(Node):
    def __init__(self) -> None:
        super().__init__("targets_launcher")
        self._load_parameters()

        self._enabled = False
        self._next_launch: Time | None = None
        self._balls: list[BallPosition] | None = None

        self._subscription = self.create_subscription(
            FreeJointStateArray, self._ball_states_topic, self._on_ball_states, 10
        )
        self._client = self.create_client(SetFreeJointState, self._set_state_service)
        self._service = self.create_service(SetBool, "~/launch", self._on_launch_request)
        self._timer = self.create_timer(CHECK_PERIOD, self._on_timer)
        self.get_logger().info("Подача шаров выключена; включить: сервис ~/launch (std_srvs/SetBool, data: true)")

    # --- параметры ---------------------------------------------------------------------------

    def _load_parameters(self) -> None:
        def declare(name: str, default, description: str):
            descriptor = ParameterDescriptor(description=description, read_only=True)
            return self.declare_parameter(name, default, descriptor).value

        interval = declare("launch_interval", [2.0, 5.0], "Наименьший и наибольший интервал между подачами, с")
        point = declare("launch_point", [-0.25, 0.0, 0.9], "Точка подачи шара (x, y, z), м")
        self._max_speed = float(declare("max_launch_speed", 0.1, "Наибольшая начальная скорость шара, м/с"))
        zone_x = declare("zone_x", [-0.60, 0.38], "Стенки рабочей зоны по x, м")
        zone_y = declare("zone_y", [-0.60, 0.60], "Стенки рабочей зоны по y, м")
        blue_bin = declare("blue_bin", [0.381, 0.531, 0.001, 0.599], "Внутренние границы синего контейнера: x_min, x_max, y_min, y_max, м")
        red_bin = declare("red_bin", [0.381, 0.531, -0.599, -0.001], "Внутренние границы красного контейнера: x_min, x_max, y_min, y_max, м")
        bin_height = float(declare("bin_height", 0.30, "Высота стенок контейнеров, м"))
        self._max_balls_in_zone = int(
            declare("max_balls_in_zone", 5, "Подача пропускается, если в рабочей зоне столько шаров или больше")
        )
        seed = int(declare("random_seed", 0, "Начальное значение генератора случайных чисел; 0 — случайное"))
        self._ball_states_topic = str(
            declare("ball_states_topic", "/free_joint_state_publisher/free_joint_states", "Topic с истинными состояниями шаров")
        )
        self._set_state_service = str(
            declare("set_state_service", "/mujoco_ros2_control_node/set_free_joint_state", "Сервис симулятора для переноса шара")
        )

        # Ошибка конфигурации останавливает узел при запуске с понятным сообщением.
        _require_length("launch_interval", interval, 2)
        _require_length("launch_point", point, 3)
        _require_length("zone_x", zone_x, 2)
        _require_length("zone_y", zone_y, 2)
        _require_length("blue_bin", blue_bin, 4)
        _require_length("red_bin", red_bin, 4)
        if not 0.0 < interval[0] <= interval[1]:
            raise ValueError("launch_interval: нужен положительный интервал, наименьшее значение не больше наибольшего")
        if self._max_speed < 0.0:
            raise ValueError("max_launch_speed не может быть отрицательной")
        if self._max_balls_in_zone < 1:
            raise ValueError("max_balls_in_zone должно быть не меньше 1")

        self._interval = (float(interval[0]), float(interval[1]))
        self._launch_point = (float(point[0]), float(point[1]), float(point[2]))
        self._bins = [Bin(*map(float, blue_bin), height=bin_height), Bin(*map(float, red_bin), height=bin_height)]
        self._zone = Zone(float(zone_x[0]), float(zone_x[1]), float(zone_y[0]), float(zone_y[1]))
        self._away = direction_from_nearest_wall(self._launch_point[0], self._launch_point[1], self._zone)
        self._rng = random.Random(seed if seed != 0 else None)

    # --- обработчики -------------------------------------------------------------------------

    def _on_ball_states(self, message: FreeJointStateArray) -> None:
        self._balls = [
            BallPosition(
                state.name,
                state.pose.pose.position.x,
                state.pose.pose.position.y,
                state.pose.pose.position.z,
            )
            for state in message.free_joints
        ]

    def _on_launch_request(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self._enabled = request.data
        if self._enabled:
            self._schedule_next()
            response.message = "подача шаров включена"
        else:
            self._next_launch = None
            response.message = "подача шаров выключена"
        response.success = True
        self.get_logger().info(response.message.capitalize())
        return response

    def _on_timer(self) -> None:
        if not self._enabled or self._next_launch is None:
            return
        if self.get_clock().now() < self._next_launch:
            return
        self._schedule_next()
        self._launch_ball()

    # --- подача ------------------------------------------------------------------------------

    def _schedule_next(self) -> None:
        delay = self._rng.uniform(*self._interval)
        self._next_launch = self.get_clock().now() + rclpy.duration.Duration(seconds=delay)

    def _launch_ball(self) -> None:
        if self._balls is None:
            self.get_logger().warning(
                f"Подача пропущена: нет данных о шарах из {self._ball_states_topic}", throttle_duration_sec=5.0
            )
            return
        in_zone = balls_in_zone(self._balls, self._zone)
        if in_zone >= self._max_balls_in_zone:
            self.get_logger().info(
                f"Подача пропущена: в рабочей зоне {in_zone} шаров", throttle_duration_sec=5.0
            )
            return
        name = choose_ball(self._rng, balls_in_bins(self._balls, self._bins))
        if name is None:
            self.get_logger().info("Подача пропущена: в контейнерах нет шаров", throttle_duration_sec=5.0)
            return
        if not self._client.service_is_ready():
            self.get_logger().warning(
                f"Подача пропущена: сервис {self._set_state_service} недоступен", throttle_duration_sec=5.0
            )
            return

        velocity = random_launch_velocity(self._rng, self._max_speed, self._away)
        state = FreeJointState()
        state.name = name
        state.pose.pose.position.x, state.pose.pose.position.y, state.pose.pose.position.z = self._launch_point
        state.twist.twist.linear.x, state.twist.twist.linear.y, state.twist.twist.linear.z = velocity
        request = SetFreeJointState.Request()
        request.free_joints = [state]
        # Ответ обрабатывается в отдельном обратном вызове: ждать его здесь нельзя,
        # обработчик таймера заблокировал бы единственный поток узла.
        future = self._client.call_async(request)
        future.add_done_callback(lambda done: self._on_launch_done(name, done))
        self.get_logger().info(
            f"Подан {name}: скорость ({velocity[0]:+.3f}, {velocity[1]:+.3f}, {velocity[2]:+.3f}) м/с"
        )

    def _on_launch_done(self, name: str, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 — сбой вызова сервиса не должен останавливать узел
            self.get_logger().error(f"Не удалось подать {name}: {error}")
            return
        if not response.success:
            self.get_logger().error(f"Симулятор отклонил перенос {name}: {response.message}")


def _require_length(name: str, value, length: int) -> None:
    if len(value) != length:
        raise ValueError(f"{name}: ожидается {length} чисел, получено {len(value)}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = TargetsLauncher()
    except ValueError as error:
        rclpy.logging.get_logger("targets_launcher").fatal(f"Неверная конфигурация: {error}")
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
