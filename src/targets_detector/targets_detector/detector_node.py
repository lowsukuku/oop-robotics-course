"""ROS 2-узел /targets_detector: имитация системы технического зрения.

В реальном роботе на месте этого узла работает узел машинного зрения с тем же
интерфейсом detected_targets. В симуляции узел получает истинные позы и скорости
шаров (topic free_joint_states плагина симулятора) и с заданной частотой публикует
только шары, пригодные для захвата: покоящиеся на столе в рабочей зоне, в порядке
обнаружения.
"""

from __future__ import annotations

import rclpy
import rclpy.logging
from ball_sorting_interfaces.msg import DetectedTarget, DetectedTargets
from mujoco_ros2_control_msgs.msg import FreeJointStateArray
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from targets_detector.detection import BallObservation, Color, DetectionConfig, DetectionQueue, detect

COLOR_CODES = {Color.BLUE: DetectedTarget.BLUE, Color.RED: DetectedTarget.RED}


class TargetsDetector(Node):
    def __init__(self) -> None:
        super().__init__("targets_detector")
        self._load_parameters()
        self._queue = DetectionQueue()
        self._detected: list[BallObservation] = []
        self._stamp = None  # время последнего измерения; до него публиковать нечего

        self._publisher = self.create_publisher(DetectedTargets, "~/detected_targets", 10)
        self._subscription = self.create_subscription(
            FreeJointStateArray, self._ball_states_topic, self._on_ball_states, 10
        )
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish)

    def _load_parameters(self) -> None:
        def declare(name: str, default, description: str):
            descriptor = ParameterDescriptor(description=description, read_only=True)
            return self.declare_parameter(name, default, descriptor).value

        defaults = DetectionConfig()
        self._publish_rate = float(declare("publish_rate", 10.0, "Частота публикации detected_targets, Гц"))
        zone_x = declare("work_zone_x", list(defaults.work_zone_x), "Границы рабочей зоны по x, м")
        zone_y = declare("work_zone_y", list(defaults.work_zone_y), "Границы рабочей зоны по y, м")
        if len(zone_x) != 2 or len(zone_y) != 2:
            raise ValueError("work_zone_x и work_zone_y: ожидается по два числа — нижняя и верхняя граница")
        self._config = DetectionConfig(
            rest_speed_threshold=float(
                declare("rest_speed_threshold", defaults.rest_speed_threshold, "Шар покоится при меньшей скорости, м/с")
            ),
            work_zone_x=(float(zone_x[0]), float(zone_x[1])),
            work_zone_y=(float(zone_y[0]), float(zone_y[1])),
            table_ball_z=float(declare("table_ball_z", defaults.table_ball_z, "Высота центра шара на столе, м")),
            table_ball_z_tolerance=float(
                declare("table_ball_z_tolerance", defaults.table_ball_z_tolerance, "Допуск высоты центра шара, м")
            ),
        )
        self._config.validate()
        if self._publish_rate <= 0.0:
            raise ValueError("publish_rate должна быть положительной")

        blue = declare("blue_balls", [f"ball_{i}" for i in range(10)], "Имена синих шаров")
        red = declare("red_balls", [f"ball_{i}" for i in range(10, 20)], "Имена красных шаров")
        if set(blue) & set(red):
            raise ValueError("blue_balls и red_balls: один шар не может быть двух цветов")
        self._colors = {name: Color.BLUE for name in blue} | {name: Color.RED for name in red}
        self._frame_id = str(declare("frame_id", "base_link", "Система координат публикуемых положений"))
        self._ball_states_topic = str(
            declare("ball_states_topic", "/free_joint_state_publisher/free_joint_states", "Topic с истинными состояниями шаров")
        )

    def _on_ball_states(self, message: FreeJointStateArray) -> None:
        """Очередь обновляется по каждому измерению, чтобы порядок отражал момент обнаружения."""
        observations = []
        for state in message.free_joints:
            if state.name not in self._colors:
                continue  # тело без известного цвета — не шар задачи
            position, velocity = state.pose.pose.position, state.twist.twist.linear
            observations.append(
                BallObservation(state.name, position.x, position.y, position.z, velocity.x, velocity.y, velocity.z)
            )
        self._detected = detect(observations, self._config, self._queue)
        self._stamp = message.header.stamp

    def _publish(self) -> None:
        if self._stamp is None:
            return
        message = DetectedTargets()
        message.header.stamp = self._stamp  # время измерения, а не публикации
        message.header.frame_id = self._frame_id
        for ball in self._detected:
            target = DetectedTarget()
            target.name = ball.name
            target.color = COLOR_CODES[self._colors[ball.name]]
            target.position.x, target.position.y, target.position.z = ball.x, ball.y, ball.z
            message.targets.append(target)
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = TargetsDetector()
    except ValueError as error:
        rclpy.logging.get_logger("targets_detector").fatal(f"Неверная конфигурация: {error}")
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
