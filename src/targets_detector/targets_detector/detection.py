"""Логика узла /targets_detector без ROS.

Имитация технического зрения отбирает шары, пригодные для захвата, и упорядочивает
их по моменту обнаружения. Здесь только чистые функции и небольшой класс очереди:
их проверяют модульные тесты, а узел (detector_node.py) преобразует сообщения.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Iterable


class Color(Enum):
    BLUE = "blue"
    RED = "red"


@dataclass(frozen=True)
class BallObservation:
    """Истинное состояние шара из симулятора: положение центра и линейная скорость."""

    name: str
    x: float
    y: float
    z: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    @property
    def speed(self) -> float:
        return math.sqrt(self.vx * self.vx + self.vy * self.vy + self.vz * self.vz)


@dataclass(frozen=True)
class DetectionConfig:
    """Пороги и границы отбора шаров."""

    rest_speed_threshold: float = 0.005  # м/с: шар покоится при меньшей скорости
    work_zone_x: tuple[float, float] = (-0.30, 0.30)  # м
    work_zone_y: tuple[float, float] = (-0.30, 0.30)  # м
    table_ball_z: float = 0.034  # м: высота центра шара, лежащего на столе
    table_ball_z_tolerance: float = 0.010  # м

    def validate(self) -> None:
        """Проверить параметры; сообщение исключения называет неверный параметр."""
        if self.rest_speed_threshold <= 0.0:
            raise ValueError("rest_speed_threshold должен быть положительным")
        for name, (low, high) in (("work_zone_x", self.work_zone_x), ("work_zone_y", self.work_zone_y)):
            if low >= high:
                raise ValueError(f"{name}: нижняя граница должна быть меньше верхней")
        if self.table_ball_z <= 0.0:
            raise ValueError("table_ball_z должен быть положительным")
        if self.table_ball_z_tolerance <= 0.0:
            raise ValueError("table_ball_z_tolerance должен быть положительным")


def is_at_rest(ball: BallObservation, config: DetectionConfig) -> bool:
    """Шар покоится: модуль линейной скорости меньше порога."""
    return ball.speed < config.rest_speed_threshold


def is_on_table(ball: BallObservation, config: DetectionConfig) -> bool:
    """Шар лежит на столе: высота центра равна радиусу шара с допуском."""
    return abs(ball.z - config.table_ball_z) <= config.table_ball_z_tolerance


def is_in_work_zone(ball: BallObservation, config: DetectionConfig) -> bool:
    """Проекция центра шара лежит в границах рабочей зоны. Это отсеивает шары в контейнерах."""
    return (
        config.work_zone_x[0] <= ball.x <= config.work_zone_x[1]
        and config.work_zone_y[0] <= ball.y <= config.work_zone_y[1]
    )


def is_graspable(ball: BallObservation, config: DetectionConfig) -> bool:
    """Шар пригоден для захвата, если одновременно выполнены все три условия."""
    return is_at_rest(ball, config) and is_on_table(ball, config) and is_in_work_zone(ball, config)


class DetectionQueue:
    """Очередь обнаруженных шаров в порядке обнаружения.

    Новый пригодный для захвата шар добавляется в конец. Шар, который перестал быть
    пригодным (покатился, поднят захватом, оказался в контейнере), удаляется; когда он
    снова остановится на столе, он обнаруживается заново и встаёт в конец очереди.
    """

    def __init__(self) -> None:
        self._names: list[str] = []

    def update(self, graspable: Iterable[str]) -> list[str]:
        """Обновить очередь по именам шаров, пригодных для захвата сейчас, и вернуть её копию."""
        current = list(graspable)
        present = set(current)
        self._names = [name for name in self._names if name in present]
        known = set(self._names)
        for name in current:
            if name not in known:
                self._names.append(name)
                known.add(name)
        return list(self._names)

    @property
    def names(self) -> list[str]:
        return list(self._names)


def detect(
    balls: Iterable[BallObservation], config: DetectionConfig, queue: DetectionQueue
) -> list[BallObservation]:
    """Отобрать пригодные для захвата шары и вернуть их в порядке обнаружения."""
    graspable = {ball.name: ball for ball in balls if is_graspable(ball, config)}
    return [graspable[name] for name in queue.update(graspable)]
