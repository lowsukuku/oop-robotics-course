cmake_minimum_required(VERSION 3.16)
project(ball_sorting_description)

find_package(ament_cmake REQUIRED)

# Описание робота (xacro): кинематика и подключение симулятора к ros2_control.
install(DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
)

# Сцена MuJoCo и модели лежат в каталоге simulation/ в корне репозитория: их открывают
# и без ROS (просмотрщик MuJoCo, скрипты проверки). Пакет устанавливает их копию рядом
# с описанием робота, сохраняя взаимное расположение файлов: сцена ссылается на модели
# относительным путём.
set(SIMULATION_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../simulation")
install(DIRECTORY "${SIMULATION_DIR}/ball_sorting" "${SIMULATION_DIR}/models"
  DESTINATION share/${PROJECT_NAME}/simulation
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
