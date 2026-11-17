# Используем официальный ROS 2 Jazzy на Ubuntu 24.04 как базу; образ уже содержит ROS, CMake, Python, pytest, GoogleTest и colcon.
FROM ros:jazzy-ros-base-noble

# Устанавливаем MuJoCo для ros2_control, контроллеры, xacro и публикацию состояния робота.
# ros2controlcli добавляет команды `ros2 control …` для проверки контроллеров.
# TRAC-IK (trac_ik_lib) — библиотека для решения обратной задачи кинематики манипулятора.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-mujoco-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-ros2controlcli \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-trac-ik-lib \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Устанавливаем Python-модуль MuJoCo для проверок сцены и моделей без ROS (скрипты в simulation/checks).
# Версию берём из заголовка библиотеки MuJoCo, которую ставит пакет mujoco_vendor: тогда физика в проверках
# и в симуляции одинакова. mjVERSION_HEADER имеет вид 3012000 — старшая версия, три цифры младшей, три цифры исправления.
# Ограничение numpy<2 сохраняет системный NumPy, с которым собраны Python-пакеты ROS 2 Jazzy.
RUN header=$(sed -n 's/^#define mjVERSION_HEADER[[:space:]]*\([0-9]*\).*/\1/p' /opt/ros/jazzy/opt/mujoco_vendor/include/mujoco/mujoco.h) \
    && mujoco_version="$((header / 1000000)).$((header / 1000 % 1000)).$((header % 1000))" \
    && pip3 install --no-cache-dir --break-system-packages "mujoco==${mujoco_version}" "numpy<2"

# Создаём группу render с типичным для Ubuntu GID 109 для доступа обычного пользователя к графическому устройству.
RUN groupadd --gid 109 render

# Используем пользователя ubuntu с UID/GID 1000, убираем административную группу и готовим ему рабочую директорию.
RUN mkdir -p /workspace \
    && chown ubuntu:ubuntu /workspace \
    && gpasswd --delete ubuntu sudo

# Добавляем путь к ROS 2 в окружение всех процессов, включая команды, запущенные через docker compose exec.
ENV PATH="/opt/ros/jazzy/bin:${PATH}"

# Загружаем полное окружение ROS 2 при открытии интерактивной оболочки.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/ubuntu/.bashrc

# Назначаем рабочей директорией примонтированную папку проекта.
WORKDIR /workspace

# Запускаем команды контейнера от обычного пользователя, а не от root.
USER ubuntu

# Открываем интерактивную оболочку по умолчанию; Compose переопределяет команду для постоянной работы контейнера.
CMD ["bash"]
