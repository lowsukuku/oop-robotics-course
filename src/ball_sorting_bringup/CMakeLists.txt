cmake_minimum_required(VERSION 3.16)
project(ball_sorting_bringup)

find_package(ament_cmake REQUIRED)

# Параметры узлов и контроллеров, launch-файлы.
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
