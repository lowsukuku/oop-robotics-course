#!/usr/bin/env python3
"""Генератор сцены MuJoCo для задачи сортировки шаров.

Сцена описана в simulation/README.md (раздел «Геометрия»). Скрипт собирает файл
scene.xml из параметров, заданных ниже: так размеры сцены лежат в одном месте,
а зависящие от них величины (наклон скосов, вершины воротника, начальные
положения шаров, keyframe) вычисляются, а не подбираются вручную.

Запуск (из любой папки):

    python3 simulation/ball_sorting/generate_scene.py

Файл scene.xml хранится в репозитории: его загружают симулятор и проверки.
После изменения параметров сцену нужно сгенерировать заново.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SceneParams:
    """Размеры сцены в метрах; система координат — основание манипулятора."""

    # Плоская часть рабочей зоны: x, y ∈ [-flat_half, flat_half], в центре — манипулятор.
    flat_half: float = 0.30
    # Границы рабочей зоны (стенки): слева и спереди/сзади — ограждение, справа — стенки контейнеров.
    zone_x_min: float = -0.60
    zone_x_max: float = 0.38
    zone_y_half: float = 0.60
    # Скосы поднимаются от плоской части к стенкам. Широкие скосы (слева, спереди, сзади)
    # и узкий скос у контейнеров заданы высотой у стенки.
    bevel_height: float = 0.034
    bevel_height_bins: float = 0.02
    # Воротник вокруг основания: радиус у стола, радиус у основания, высота.
    collar_radius: float = 0.22
    collar_top_radius: float = 0.075
    collar_height: float = 0.02
    collar_segments: int = 48
    # Контейнеры: внутренний размер по x, высота и толщина стенок; по y контейнер занимает половину зоны.
    bin_inner_x: float = 0.150
    bin_height: float = 0.300
    bin_wall: float = 0.001
    # Ограждение.
    fence_height: float = 0.9
    fence_thickness: float = 0.01
    # Шары.
    ball_radius: float = 0.034
    ball_mass: float = 0.28
    balls_per_color: int = 10
    ball_start_height: float = 0.045
    # Контакт шаров: трение скольжения, кручения и качения; жёсткость контакта.
    ball_friction: tuple[float, float, float] = (0.6, 0.002, 0.0006)
    ball_solref: tuple[float, float] = (0.004, 1.0)
    # Начальная поза манипулятора (keyframe initial), рад: рука сложена над плоской частью зоны
    # со стороны контейнеров, захват смотрит вниз, TCP в точке (0.25, 0, 0.45).
    initial_arm: tuple[float, ...] = (2.5759, -2.1611, 1.6805, -1.0902, -1.5708, -0.5657)
    # Равновесное положение суставов раскрытого захвата (8 суставов 2F-85), найдено в симуляции.
    initial_gripper: tuple[float, ...] = (0.00293, 0.00016, 0.00319, -0.00343, 0.00293, 0.00016, 0.0032, -0.00344)

    @property
    def bin_x_min(self) -> float:
        return self.zone_x_max

    @property
    def bin_x_max(self) -> float:
        return self.zone_x_max + self.bin_inner_x + 2 * self.bin_wall

    @property
    def bin_inner_y(self) -> float:
        return self.zone_y_half - 2 * self.bin_wall

    def ball_start_positions(self) -> list[tuple[float, float, float]]:
        """Начальные центры шаров: ball_0…9 над синим контейнером, ball_10…19 — над красным."""
        columns = 2
        rows = self.balls_per_color // columns
        x_center = 0.5 * (self.bin_x_min + self.bin_x_max)
        x_pitch = self.bin_inner_x / columns
        y_pitch = self.bin_inner_y / rows
        positions = []
        for side in (+1, -1):  # +1 — синий (y > 0), -1 — красный (y < 0)
            for index in range(self.balls_per_color):
                column, row = index % columns, index // columns
                x = x_center + (column - (columns - 1) / 2) * x_pitch
                y = side * (self.bin_wall + (row + 0.5) * y_pitch)
                positions.append((x, y, self.ball_start_height))
        return positions


def _fmt(value: float) -> str:
    """Короткая запись числа без «-0» и лишних нулей."""
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def _vec(*values: float) -> str:
    return " ".join(_fmt(v) for v in values)


def _collar_vertices(p: SceneParams) -> str:
    """Вершины усечённого конуса. Нижнее кольцо опущено под стол вдоль образующей,
    чтобы поверхность конуса пересекала стол ровно на радиусе воротника без ступеньки."""
    sink = 0.01
    slope = (p.collar_radius - p.collar_top_radius) / p.collar_height
    bottom_radius = p.collar_radius + sink * slope
    rings = ((bottom_radius, -sink), (p.collar_top_radius, p.collar_height))
    vertices = []
    for radius, z in rings:
        for k in range(p.collar_segments):
            angle = 2 * math.pi * k / p.collar_segments
            vertices.append(_vec(radius * math.cos(angle), radius * math.sin(angle), z))
    return "  ".join(vertices)


def _bevel(name: str, inner: float, outer: float, height: float, along: tuple[float, float], axis: str) -> str:
    """Скос — параллелепипед, повёрнутый так, что его верхняя грань поднимается от плоской части
    (координата inner, высота 0) к стенке (координата outer, высота height).

    axis — ось, вдоль которой скос поднимается ("x" или "y"); along — границы скоса по другой оси.
    """
    width = abs(outer - inner)
    direction = 1.0 if outer > inner else -1.0
    angle = math.atan2(height, width)
    length = math.hypot(width, height)  # длина наклонной грани
    thickness = 0.02  # толщина бруска; он уходит под стол и снаружи не виден
    # Центр бруска: середина наклонной грани, смещённая на половину толщины вниз по нормали.
    centre_out = (inner + outer) / 2 + direction * math.sin(angle) * thickness / 2
    centre_z = height / 2 - math.cos(angle) * thickness / 2
    centre_along = (along[0] + along[1]) / 2
    half_along = (along[1] - along[0]) / 2
    half = angle / 2
    if axis == "x":
        pos = (centre_out, centre_along, centre_z)
        size = (length / 2, half_along, thickness / 2)
        quat = (math.cos(half), 0.0, -direction * math.sin(half), 0.0)
    else:
        pos = (centre_along, centre_out, centre_z)
        size = (half_along, length / 2, thickness / 2)
        quat = (math.cos(half), direction * math.sin(half), 0.0, 0.0)
    return f'    <geom name="{name}" class="bevel" pos="{_vec(*pos)}" size="{_vec(*size)}" quat="{_vec(*quat)}"/>'


def _bevels(p: SceneParams) -> list[str]:
    """Четыре скоса от плоской части к стенкам; в углах скосы перекрываются."""
    span_x = (p.zone_x_min, p.zone_x_max)
    span_y = (-p.zone_y_half, p.zone_y_half)
    return [
        _bevel("bevel_left", -p.flat_half, p.zone_x_min, p.bevel_height, span_y, "x"),
        _bevel("bevel_right", p.flat_half, p.zone_x_max, p.bevel_height_bins, span_y, "x"),
        _bevel("bevel_front", p.flat_half, p.zone_y_half, p.bevel_height, span_x, "y"),
        _bevel("bevel_back", -p.flat_half, -p.zone_y_half, p.bevel_height, span_x, "y"),
    ]


def _bins(p: SceneParams) -> list[str]:
    """Два контейнера справа от рабочей зоны: стенки толщиной bin_wall, дно — поверхность стола."""
    lines = []
    half_h = p.bin_height / 2
    x_min, x_max = p.bin_x_min, p.bin_x_max
    x_mid = 0.5 * (x_min + x_max)
    half_x = 0.5 * (x_max - x_min)
    outer_y = p.bin_inner_y + 2 * p.bin_wall
    for name, sign in (("blue", +1), ("red", -1)):
        y_mid = sign * outer_y / 2
        walls = (
            (f"bin_{name}_front", (x_min + p.bin_wall / 2, y_mid, half_h), (p.bin_wall / 2, outer_y / 2, half_h)),
            (f"bin_{name}_back", (x_max - p.bin_wall / 2, y_mid, half_h), (p.bin_wall / 2, outer_y / 2, half_h)),
            (f"bin_{name}_inner", (x_mid, sign * p.bin_wall / 2, half_h), (half_x, p.bin_wall / 2, half_h)),
            (f"bin_{name}_outer", (x_mid, sign * (outer_y - p.bin_wall / 2), half_h), (half_x, p.bin_wall / 2, half_h)),
        )
        for wall_name, pos, size in walls:
            lines.append(
                f'    <geom name="{wall_name}" class="bin_{name}" pos="{_vec(*pos)}" size="{_vec(*size)}"/>'
            )
        lines.append(
            f'    <geom name="bin_{name}_floor" class="marking" material="bin_{name}_floor" '
            f'pos="{_vec(x_mid, y_mid, 0.0005)}" size="{_vec(half_x, outer_y / 2, 0.0005)}"/>'
        )
    return lines


def _fence(p: SceneParams) -> list[str]:
    """Ограждение по внешнему периметру рабочей зоны и контейнеров."""
    t = p.fence_thickness
    half_h = p.fence_height / 2
    x_min, x_max = p.zone_x_min, p.bin_x_max
    y_half = p.zone_y_half
    x_mid = 0.5 * (x_min + x_max)
    half_x = 0.5 * (x_max - x_min) + t
    walls = (
        ("fence_left", (x_min - t / 2, 0, half_h), (t / 2, y_half + t, half_h)),
        ("fence_right", (x_max + t / 2, 0, half_h), (t / 2, y_half + t, half_h)),
        ("fence_front", (x_mid, y_half + t / 2, half_h), (half_x, t / 2, half_h)),
        ("fence_back", (x_mid, -y_half - t / 2, half_h), (half_x, t / 2, half_h)),
    )
    return [
        f'    <geom name="{name}" class="fence" pos="{_vec(*pos)}" size="{_vec(*size)}"/>'
        for name, pos, size in walls
    ]


def _balls(p: SceneParams) -> list[str]:
    lines = []
    for index, (x, y, z) in enumerate(p.ball_start_positions()):
        color = "blue" if index < p.balls_per_color else "red"
        lines.append(
            f'    <body name="ball_{index}" pos="{_vec(x, y, z)}">'
            f'<freejoint name="ball_{index}_joint"/>'
            f'<geom name="ball_{index}_geom" class="ball" material="ball_{color}"/></body>'
        )
    return lines


def _keyframe(p: SceneParams) -> list[str]:
    rows = ["         " + _vec(*p.initial_arm), "         " + _vec(*p.initial_gripper)]
    rows += ["         " + _vec(x, y, z, 1, 0, 0, 0) for x, y, z in p.ball_start_positions()]
    ctrl = _vec(*p.initial_arm, 0.0)
    return [
        '    <key name="initial"',
        '         qpos="',
        *[row.strip().rjust(len(row)) for row in rows[:1]],
        *rows[1:],
        '         "',
        f'         ctrl="{ctrl}"/>',
    ]


TEMPLATE = """\
<!--
  Сцена задачи сортировки шаров: UR5e + Robotiq 2F-85, рабочая зона, контейнеры, 20 шаров.
  Описание — simulation/README.md. Мировая система координат совпадает с системой основания
  манипулятора: начало — центр нижней поверхности основания, +x — вправо (к контейнерам), +y — вперёд.

  Файл сгенерирован скриптом generate_scene.py. Размеры сцены меняйте в нём, а не здесь.
-->
<mujoco model="ball_sorting">
  <compiler angle="radian" meshdir="../models/mujoco_menagerie/universal_robots_ur5e/assets" autolimits="true"/>

  <option integrator="implicitfast" cone="elliptic" impratio="10"/>

  <statistic center="0.08 0 0.25" extent="0.9"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0.1 0.1 0.1"/>
    <global azimuth="120" elevation="-35" offwidth="1280" offheight="960"/>
  </visual>

  <default>
    <default class="ur5e">
      <material specular="0.5" shininess="0.25"/>
      <joint axis="0 1 0" range="-6.28319 6.28319" armature="0.1"/>
      <position ctrlrange="-6.2831 6.2831" kp="4000" dampratio="1" forcerange="-150 150"/>
      <default class="size3">
        <default class="size3_limited">
          <joint range="-3.1415 3.1415"/>
          <position ctrlrange="-3.1415 3.1415"/>
        </default>
      </default>
      <default class="size1">
        <position kp="1000" forcerange="-60 60"/>
      </default>
      <default class="visual">
        <geom type="mesh" contype="0" conaffinity="0" group="2"/>
      </default>
      <default class="collision">
        <geom type="capsule" group="3"/>
        <default class="eef_collision">
          <geom type="cylinder"/>
        </default>
      </default>
      <site size="0.001" rgba="0.5 0.5 0.5 0.3" group="4"/>
    </default>

    <!-- Элементы сцены -->
    <default class="bevel">
      <geom type="box" rgba="0.62 0.66 0.72 1"/>
    </default>
    <default class="fence">
      <geom type="box" rgba="0.75 0.85 0.95 0.12"/>
    </default>
    <default class="bin_blue">
      <geom type="box" rgba="0.15 0.35 0.9 0.35"/>
    </default>
    <default class="bin_red">
      <geom type="box" rgba="0.9 0.2 0.15 0.35"/>
    </default>
    <default class="marking">
      <geom type="box" contype="0" conaffinity="0"/>
    </default>
    <!-- Шар: condim=6 включает трение качения, без него шар на ровном столе не останавливается -->
    <default class="ball">
      <geom type="sphere" size="{ball_radius}" mass="{ball_mass}" condim="6"
            friction="{ball_friction}" solref="{ball_solref}"/>
    </default>
  </default>

  <asset>
    <model name="robotiq_2f85" file="../models/mujoco_menagerie/robotiq_2f85/2f85.xml"/>
    <material class="ur5e" name="black" rgba="0.033 0.033 0.033 1"/>
    <material class="ur5e" name="jointgray" rgba="0.278 0.278 0.278 1"/>
    <material class="ur5e" name="linkgray" rgba="0.82 0.82 0.82 1"/>
    <material class="ur5e" name="urblue" rgba="0.49 0.678 0.8 1"/>
    <material name="table" rgba="0.86 0.87 0.88 1"/>
    <material name="collar" rgba="0.62 0.66 0.72 1"/>
    <material name="ball_blue" rgba="0.1 0.3 0.95 1"/>
    <material name="ball_red" rgba="0.95 0.15 0.1 1"/>
    <material name="bin_blue_floor" rgba="0.15 0.35 0.9 0.25"/>
    <material name="bin_red_floor" rgba="0.9 0.2 0.15 0.25"/>

    <mesh file="base_0.obj"/>
    <mesh file="base_1.obj"/>
    <mesh file="shoulder_0.obj"/>
    <mesh file="shoulder_1.obj"/>
    <mesh file="shoulder_2.obj"/>
    <mesh file="upperarm_0.obj"/>
    <mesh file="upperarm_1.obj"/>
    <mesh file="upperarm_2.obj"/>
    <mesh file="upperarm_3.obj"/>
    <mesh file="forearm_0.obj"/>
    <mesh file="forearm_1.obj"/>
    <mesh file="forearm_2.obj"/>
    <mesh file="forearm_3.obj"/>
    <mesh file="wrist1_0.obj"/>
    <mesh file="wrist1_1.obj"/>
    <mesh file="wrist1_2.obj"/>
    <mesh file="wrist2_0.obj"/>
    <mesh file="wrist2_1.obj"/>
    <mesh file="wrist2_2.obj"/>
    <mesh file="wrist3.obj"/>

    <!-- Воротник вокруг основания: усечённый конус, заданный вершинами (выпуклая оболочка) -->
    <mesh name="collar" vertex="{collar_vertices}"/>
  </asset>

  <worldbody>
    <light pos="0 -1 2" dir="0 0.5 -1" directional="true"/>

    <!-- Стол: поверхность z = 0 -->
    <geom name="table" type="plane" size="2 2 0.05" material="table"/>

    <!-- Воротник вокруг основания манипулятора -->
    <geom name="collar" type="mesh" mesh="collar" material="collar"/>

    <!-- Скосы по периметру рабочей зоны -->
{bevels}

    <!-- Контейнеры: синий (y > 0) и красный (y < 0) -->
{bins}

    <!-- Ограждение -->
{fence}

    <!-- Манипулятор UR5e (MuJoCo Menagerie); система base повёрнута на 180° вокруг z, как в исходной модели -->
    <body name="base" quat="0 0 0 -1" childclass="ur5e">
      <inertial mass="4.0" pos="0 0 0" diaginertia="0.00443333156 0.00443333156 0.0072"/>
      <geom mesh="base_0" material="black" class="visual"/>
      <geom mesh="base_1" material="jointgray" class="visual"/>
      <geom name="base_collision" class="eef_collision" size="0.075 0.045" pos="0 0 0.045"/>
      <body name="shoulder_link" pos="0 0 0.163" gravcomp="1">
        <inertial mass="3.7" pos="0 0 0" diaginertia="0.0102675 0.0102675 0.00666"/>
        <joint name="shoulder_pan_joint" class="size3" axis="0 0 1"/>
        <geom mesh="shoulder_0" material="urblue" class="visual"/>
        <geom mesh="shoulder_1" material="black" class="visual"/>
        <geom mesh="shoulder_2" material="jointgray" class="visual"/>
        <geom class="collision" size="0.06 0.06" pos="0 0 -0.04"/>
        <body name="upper_arm_link" pos="0 0.138 0" quat="1 0 1 0" gravcomp="1">
          <inertial mass="8.393" pos="0 0 0.2125" diaginertia="0.133886 0.133886 0.0151074"/>
          <joint name="shoulder_lift_joint" class="size3"/>
          <geom mesh="upperarm_0" material="linkgray" class="visual"/>
          <geom mesh="upperarm_1" material="black" class="visual"/>
          <geom mesh="upperarm_2" material="jointgray" class="visual"/>
          <geom mesh="upperarm_3" material="urblue" class="visual"/>
          <geom class="collision" pos="0 -0.04 0" quat="1 1 0 0" size="0.06 0.06"/>
          <geom class="collision" size="0.05 0.2" pos="0 0 0.2"/>
          <body name="forearm_link" pos="0 -0.131 0.425" gravcomp="1">
            <inertial mass="2.275" pos="0 0 0.196" diaginertia="0.0311796 0.0311796 0.004095"/>
            <joint name="elbow_joint" class="size3_limited"/>
            <geom mesh="forearm_0" material="urblue" class="visual"/>
            <geom mesh="forearm_1" material="linkgray" class="visual"/>
            <geom mesh="forearm_2" material="black" class="visual"/>
            <geom mesh="forearm_3" material="jointgray" class="visual"/>
            <geom class="collision" pos="0 0.08 0" quat="1 1 0 0" size="0.055 0.06"/>
            <geom class="collision" size="0.038 0.19" pos="0 0 0.2"/>
            <body name="wrist_1_link" pos="0 0 0.392" quat="1 0 1 0" gravcomp="1">
              <inertial mass="1.219" pos="0 0.127 0" diaginertia="0.0025599 0.0025599 0.0021942"/>
              <joint name="wrist_1_joint" class="size1"/>
              <geom mesh="wrist1_0" material="black" class="visual"/>
              <geom mesh="wrist1_1" material="urblue" class="visual"/>
              <geom mesh="wrist1_2" material="jointgray" class="visual"/>
              <geom class="collision" pos="0 0.05 0" quat="1 1 0 0" size="0.04 0.07"/>
              <body name="wrist_2_link" pos="0 0.127 0" gravcomp="1">
                <inertial mass="1.219" pos="0 0 0.1" diaginertia="0.0025599 0.0025599 0.0021942"/>
                <joint name="wrist_2_joint" axis="0 0 1" class="size1"/>
                <geom mesh="wrist2_0" material="black" class="visual"/>
                <geom mesh="wrist2_1" material="urblue" class="visual"/>
                <geom mesh="wrist2_2" material="jointgray" class="visual"/>
                <geom class="collision" size="0.04 0.06" pos="0 0 0.04"/>
                <geom class="collision" pos="0 0.02 0.1" quat="1 1 0 0" size="0.04 0.04"/>
                <body name="wrist_3_link" pos="0 0 0.1" gravcomp="1">
                  <inertial mass="0.1889" pos="0 0.0771683 0" quat="1 0 0 1"
                    diaginertia="0.000132134 9.90863e-05 9.90863e-05"/>
                  <joint name="wrist_3_joint" class="size1"/>
                  <geom material="linkgray" mesh="wrist3" class="visual"/>
                  <geom class="eef_collision" pos="0 0.08 0" quat="1 1 0 0" size="0.04 0.02"/>
                  <site name="attachment_site" pos="0 0.1 0" quat="-1 1 0 0"/>
                  <!-- Захват Robotiq 2F-85; точка tcp задана в его модели (gripper_tcp) -->
                  <frame pos="0 0.1 0" quat="-0.7071068 0.7071068 0 0">
                    <attach model="robotiq_2f85" body="base_mount" prefix="gripper_"/>
                  </frame>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- Шары: ball_0…ball_9 синие, ball_10…ball_19 красные; имя тела — идентификатор шара -->
{balls}
  </worldbody>

  <contact>
    <!-- Основание и плечо соприкасаются конструктивно -->
    <exclude body1="base" body2="shoulder_link"/>
    <exclude body1="world" body2="shoulder_link"/>
  </contact>

  <actuator>
    <position class="size3" name="shoulder_pan_joint" joint="shoulder_pan_joint"/>
    <position class="size3" name="shoulder_lift_joint" joint="shoulder_lift_joint"/>
    <position class="size3_limited" name="elbow_joint" joint="elbow_joint"/>
    <position class="size1" name="wrist_1_joint" joint="wrist_1_joint"/>
    <position class="size1" name="wrist_2_joint" joint="wrist_2_joint"/>
    <position class="size1" name="wrist_3_joint" joint="wrist_3_joint"/>
  </actuator>

  <keyframe>
    <!-- Начальное состояние: манипулятор сложен, захват раскрыт, шары над дном контейнеров.
         qpos: 6 суставов руки, 8 суставов захвата, затем по 7 чисел на шар (положение и кватернион). -->
{keyframe}
  </keyframe>
</mujoco>
"""


def build_scene(params: SceneParams) -> str:
    return TEMPLATE.format(
        ball_radius=_fmt(params.ball_radius),
        ball_mass=_fmt(params.ball_mass),
        ball_friction=_vec(*params.ball_friction),
        ball_solref=_vec(*params.ball_solref),
        collar_vertices=_collar_vertices(params),
        bevels="\n".join(_bevels(params)),
        bins="\n".join(_bins(params)),
        fence="\n".join(_fence(params)),
        balls="\n".join(_balls(params)),
        keyframe="\n".join(_keyframe(params)),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent / "scene.xml",
        help="путь к создаваемому файлу сцены",
    )
    args = parser.parse_args()
    args.output.write_text(build_scene(SceneParams()), encoding="utf-8")
    print(f"Сцена записана в {args.output}")


if __name__ == "__main__":
    main()
