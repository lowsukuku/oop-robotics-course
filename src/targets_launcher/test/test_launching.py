import math
import random

import pytest

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

BLUE_BIN = Bin(x_min=0.381, x_max=0.531, y_min=0.001, y_max=0.599, height=0.30)
RED_BIN = Bin(x_min=0.381, x_max=0.531, y_min=-0.599, y_max=-0.001, height=0.30)
ZONE = Zone(x_min=-0.60, x_max=0.38, y_min=-0.60, y_max=0.60)


def test_ball_on_bin_floor_is_in_bin():
    assert BLUE_BIN.contains(BallPosition("ball_0", 0.42, 0.3, 0.034))


def test_ball_on_second_layer_is_still_in_bin():
    assert BLUE_BIN.contains(BallPosition("ball_0", 0.42, 0.3, 0.095))


def test_ball_above_bin_walls_is_not_in_bin():
    # Шар несут над контейнером или он только что отпущен на высоте сброса.
    assert not BLUE_BIN.contains(BallPosition("ball_0", 0.44, 0.175, 0.42))


def test_ball_in_work_zone_is_not_in_bin():
    assert not BLUE_BIN.contains(BallPosition("ball_0", 0.25, 0.3, 0.034))


def test_balls_in_bins_selects_only_balls_inside_any_bin():
    balls = [
        BallPosition("ball_0", 0.42, 0.30, 0.034),  # синий контейнер
        BallPosition("ball_1", -0.25, 0.00, 0.034),  # рабочая зона
        BallPosition("ball_12", 0.50, -0.20, 0.034),  # красный контейнер
        BallPosition("ball_13", 0.10, 0.25, 0.364),  # в захвате над столом
        BallPosition("ball_3", 0.45, -0.45, 0.034),  # синий шар в красном контейнере — тоже кандидат
    ]
    assert balls_in_bins(balls, [BLUE_BIN, RED_BIN]) == ["ball_0", "ball_12", "ball_3"]


def test_balls_in_bins_is_empty_when_bins_are_empty():
    balls = [BallPosition("ball_0", 0.0, 0.25, 0.034)]
    assert balls_in_bins(balls, [BLUE_BIN, RED_BIN]) == []


def test_bin_rejects_inverted_bounds():
    with pytest.raises(ValueError):
        Bin(x_min=0.5, x_max=0.4, y_min=0.0, y_max=0.3, height=0.3)


@pytest.mark.parametrize(
    "point, expected",
    [
        ((-0.25, 0.0), (1.0, 0.0)),  # точка подачи: ближе всего левая стенка
        ((0.30, 0.0), (-1.0, 0.0)),  # у стенки контейнеров
        ((0.0, 0.55), (0.0, -1.0)),
        ((-0.1, -0.55), (0.0, 1.0)),
    ],
)
def test_direction_from_nearest_wall(point, expected):
    assert direction_from_nearest_wall(*point, ZONE) == expected


def test_launch_velocity_is_bounded_and_points_down_and_away_from_wall():
    rng = random.Random(7)
    away = (1.0, 0.0)
    for _ in range(1000):
        vx, vy, vz = random_launch_velocity(rng, 0.1, away)
        assert math.sqrt(vx * vx + vy * vy + vz * vz) <= 0.1 + 1e-12
        assert vz <= 0.0
        assert vx * away[0] + vy * away[1] >= -1e-12


def test_launch_velocity_follows_away_direction():
    rng = random.Random(3)
    away = (0.0, -1.0)
    for _ in range(200):
        _, vy, _ = random_launch_velocity(rng, 0.1, away)
        assert vy <= 1e-12


def test_launch_velocity_covers_the_whole_speed_range():
    rng = random.Random(11)
    speeds = [math.hypot(*random_launch_velocity(rng, 0.1, (1.0, 0.0))) for _ in range(2000)]
    assert min(speeds) < 0.01
    assert max(speeds) > 0.09


def test_zero_max_speed_gives_zero_velocity():
    assert random_launch_velocity(random.Random(1), 0.0, (1.0, 0.0)) == pytest.approx((0.0, 0.0, 0.0))


def test_choose_ball_returns_none_without_candidates():
    assert choose_ball(random.Random(1), []) is None


def test_choose_ball_picks_every_candidate_eventually():
    rng = random.Random(5)
    candidates = ["ball_0", "ball_7", "ball_15"]
    assert {choose_ball(rng, candidates) for _ in range(100)} == set(candidates)


def test_balls_in_zone_counts_resting_falling_and_held_balls():
    balls = [
        BallPosition("ball_0", 0.1, 0.2, 0.034),  # лежит на плоской части
        BallPosition("ball_1", -0.5, 0.0, 0.040),  # катится по скосу
        BallPosition("ball_2", -0.25, 0.0, 0.9),  # только что подан
        BallPosition("ball_3", 0.25, 0.1, 0.364),  # в захвате над зоной
        BallPosition("ball_4", 0.45, 0.3, 0.034),  # в синем контейнере
        BallPosition("ball_5", 0.44, -0.175, 0.42),  # отпущен над красным контейнером
    ]
    assert balls_in_zone(balls, ZONE) == 4


def test_balls_in_zone_of_empty_list_is_zero():
    assert balls_in_zone([], ZONE) == 0
