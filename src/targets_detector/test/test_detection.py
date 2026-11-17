import pytest

from targets_detector.detection import (
    BallObservation,
    DetectionConfig,
    DetectionQueue,
    detect,
    is_at_rest,
    is_graspable,
    is_in_work_zone,
    is_on_table,
)

CONFIG = DetectionConfig()


def resting(name: str, x: float = 0.25, y: float = 0.0) -> BallObservation:
    return BallObservation(name, x, y, 0.034)


def rolling(name: str, x: float = 0.25, y: float = 0.0) -> BallObservation:
    return BallObservation(name, x, y, 0.034, vx=0.05)


# --- три условия отбора -------------------------------------------------------------------


def test_slow_ball_is_at_rest():
    assert is_at_rest(BallObservation("b", 0.2, 0.0, 0.034, vx=0.003, vy=0.003), CONFIG)


def test_ball_at_threshold_speed_is_not_at_rest():
    assert not is_at_rest(BallObservation("b", 0.2, 0.0, 0.034, vx=0.005), CONFIG)


def test_speed_uses_all_three_components():
    falling = BallObservation("b", 0.2, 0.0, 0.034, vz=-0.5)
    assert not is_at_rest(falling, CONFIG)


@pytest.mark.parametrize("z, expected", [(0.034, True), (0.0335, True), (0.044, True), (0.0445, False), (0.364, False)])
def test_on_table_by_centre_height(z, expected):
    assert is_on_table(BallObservation("b", 0.2, 0.0, z), CONFIG) is expected


@pytest.mark.parametrize(
    "x, y, expected",
    [
        (0.0, 0.25, True),
        (0.30, -0.30, True),  # граница зоны входит в зону
        (0.31, 0.0, False),  # скос
        (0.456, 0.3, False),  # контейнер: высота центра как на столе, но вне зоны
        (0.0, -0.45, False),
    ],
)
def test_in_work_zone(x, y, expected):
    assert is_in_work_zone(BallObservation("b", x, y, 0.034), CONFIG) is expected


def test_ball_resting_in_bin_is_not_graspable():
    assert not is_graspable(BallObservation("ball_3", 0.456, 0.3, 0.034), CONFIG)


def test_ball_held_still_above_table_is_not_graspable():
    assert not is_graspable(BallObservation("ball_3", 0.2, 0.1, 0.364), CONFIG)


def test_rolling_ball_is_not_graspable():
    assert not is_graspable(rolling("ball_3"), CONFIG)


def test_resting_ball_on_table_in_zone_is_graspable():
    assert is_graspable(resting("ball_3"), CONFIG)


# --- порядок обнаружения ------------------------------------------------------------------


def test_new_detection_goes_to_the_end():
    queue = DetectionQueue()
    assert queue.update(["ball_5"]) == ["ball_5"]
    assert queue.update(["ball_2", "ball_5"]) == ["ball_5", "ball_2"]
    assert queue.update(["ball_9", "ball_2", "ball_5"]) == ["ball_5", "ball_2", "ball_9"]


def test_ball_that_moved_and_stopped_again_goes_to_the_end():
    queue = DetectionQueue()
    queue.update(["ball_1"])
    queue.update(["ball_1", "ball_2"])
    assert queue.update(["ball_2"]) == ["ball_2"]  # ball_1 покатился
    assert queue.update(["ball_1", "ball_2"]) == ["ball_2", "ball_1"]  # и снова остановился


def test_removed_ball_does_not_reorder_others():
    queue = DetectionQueue()
    for names in (["a"], ["a", "b"], ["a", "b", "c"]):
        queue.update(names)
    assert queue.update(["a", "c"]) == ["a", "c"]


def test_simultaneous_detections_keep_message_order():
    assert DetectionQueue().update(["ball_7", "ball_3"]) == ["ball_7", "ball_3"]


def test_duplicate_names_are_counted_once():
    assert DetectionQueue().update(["ball_1", "ball_1"]) == ["ball_1"]


def test_detect_filters_and_orders_observations():
    queue = DetectionQueue()
    first = [resting("ball_4", 0.25, 0.0), rolling("ball_6", -0.25, 0.0)]
    assert [ball.name for ball in detect(first, CONFIG, queue)] == ["ball_4"]

    second = [resting("ball_6", -0.26, 0.01), resting("ball_4", 0.25, 0.0)]
    detected = detect(second, CONFIG, queue)
    assert [ball.name for ball in detected] == ["ball_4", "ball_6"]
    assert detected[1].x == pytest.approx(-0.26)  # публикуется текущее положение


def test_detect_scenario_ball_hit_by_another():
    """Покоящийся шар получает удар, катится и останавливается в другом месте."""
    queue = DetectionQueue()
    detect([resting("ball_1"), resting("ball_2", 0.0, 0.25)], CONFIG, queue)
    moving = detect([rolling("ball_1"), resting("ball_2", 0.0, 0.25)], CONFIG, queue)
    assert [ball.name for ball in moving] == ["ball_2"]
    stopped = detect([resting("ball_1", 0.27, 0.05), resting("ball_2", 0.0, 0.25)], CONFIG, queue)
    assert [ball.name for ball in stopped] == ["ball_2", "ball_1"]


# --- проверка конфигурации ----------------------------------------------------------------


def test_default_config_is_valid():
    CONFIG.validate()


@pytest.mark.parametrize(
    "kwargs, parameter",
    [
        ({"rest_speed_threshold": 0.0}, "rest_speed_threshold"),
        ({"work_zone_x": (0.3, -0.3)}, "work_zone_x"),
        ({"work_zone_y": (0.1, 0.1)}, "work_zone_y"),
        ({"table_ball_z": -0.034}, "table_ball_z"),
        ({"table_ball_z_tolerance": 0.0}, "table_ball_z_tolerance"),
    ],
)
def test_invalid_config_names_the_parameter(kwargs, parameter):
    with pytest.raises(ValueError, match=parameter):
        DetectionConfig(**kwargs).validate()
