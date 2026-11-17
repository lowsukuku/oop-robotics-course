"""Вычисления узла /targets_launcher без ROS.

Здесь только чистые функции и простые типы данных: какие шары лежат в контейнерах
и с какой скоростью запустить очередной шар. Их легко проверить модульными тестами,
а узел (launcher_node.py) отвечает только за обмен сообщениями.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Iterable, Sequence

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class BallPosition:
    """Истинное положение центра шара из симулятора."""

    name: str
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Bin:
    """Внутренний объём контейнера: границы в плоскости XY и высота стенок."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    height: float

    def __post_init__(self) -> None:
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError("границы контейнера заданы неверно: минимум должен быть меньше максимума")
        if self.height <= 0.0:
            raise ValueError("высота контейнера должна быть положительной")

    def contains(self, ball: BallPosition) -> bool:
        """Шар внутри контейнера: проекция центра в его границах, центр не выше стенок."""
        return (
            self.x_min <= ball.x <= self.x_max
            and self.y_min <= ball.y <= self.y_max
            and ball.z <= self.height
        )


@dataclass(frozen=True)
class Zone:
    """Границы рабочей зоны в плоскости XY (стенки, от которых отсчитывается направление подачи)."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self) -> None:
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError("границы рабочей зоны заданы неверно: минимум должен быть меньше максимума")

    def contains(self, ball: BallPosition) -> bool:
        """Проекция центра шара лежит между стенками зоны (на любой высоте)."""
        return self.x_min <= ball.x <= self.x_max and self.y_min <= ball.y <= self.y_max


def balls_in_zone(balls: Iterable[BallPosition], zone: Zone) -> int:
    """Сколько шаров находится в рабочей зоне: лежат, катятся, падают или удерживаются захватом над ней."""
    return sum(1 for ball in balls if zone.contains(ball))


def balls_in_bins(balls: Iterable[BallPosition], bins: Sequence[Bin]) -> list[str]:
    """Имена шаров, лежащих в любом из контейнеров.

    Только такие шары можно подавать: остальные находятся в рабочей зоне, летят
    или переносятся манипулятором.
    """
    return [ball.name for ball in balls if any(bin_.contains(ball) for bin_ in bins)]


def direction_from_nearest_wall(x: float, y: float, zone: Zone) -> tuple[float, float]:
    """Единичный вектор в плоскости XY, направленный от ближайшей к точке стенки внутрь зоны."""
    distances = (
        (x - zone.x_min, (1.0, 0.0)),
        (zone.x_max - x, (-1.0, 0.0)),
        (y - zone.y_min, (0.0, 1.0)),
        (zone.y_max - y, (0.0, -1.0)),
    )
    return min(distances, key=lambda item: item[0])[1]


def random_launch_velocity(rng: random.Random, max_speed: float, away: tuple[float, float]) -> Vector3:
    """Случайная начальная скорость шара.

    Модуль — от 0 до max_speed. Направление — под произвольным углом вниз и от стенки:
    вертикальная составляющая не положительна, а проекция на направление away
    (от ближайшей стенки) не отрицательна.
    """
    if max_speed < 0.0:
        raise ValueError("наибольшая скорость подачи не может быть отрицательной")
    speed = rng.uniform(0.0, max_speed)
    descent = rng.uniform(0.0, math.pi / 2)  # угол вниз от горизонтали
    turn = rng.uniform(-math.pi / 2, math.pi / 2)  # отклонение от направления away в плоскости XY
    horizontal = speed * math.cos(descent)
    cos_turn, sin_turn = math.cos(turn), math.sin(turn)
    return (
        horizontal * (away[0] * cos_turn - away[1] * sin_turn),
        horizontal * (away[0] * sin_turn + away[1] * cos_turn),
        -speed * math.sin(descent),
    )


def choose_ball(rng: random.Random, candidates: Sequence[str]) -> str | None:
    """Случайный шар из кандидатов; None, если подавать нечего."""
    if not candidates:
        return None
    return rng.choice(list(candidates))
