import time
import random
import math
import matplotlib.pyplot as plt

from obstacle_avoidance import DroneController


def simulate_forest_scenario():
  """
  Симуляция полёта дрона к лесополосе с интерактивной визуализацией.

  В одном окне matplotlib на каждом шаге рисуется новый "вид" лидара вокруг дрона.
  График обновляется в реальном времени без блокировки окна.

  Логика:
    - Лес имеет высоту 15 м и располагается в области x ∈ [0, -15].
    - Дрон начинает на высоте 10 м и при x=30 (далеко перед лесом).
    - Каждый цикл:
      1. Генерируются 500 точек лидара, при этом в переднем секторе (±60°)
         учитываются положения леса и высота дрона.
      2. Генерируются показания дальномеров вниз и вверх (range_down, range_up)
         с небольшим шумом, учитывая положение дрона относительно леса.
      3. Вызывается алгоритм облёта (normal_flight_obstacle_avoidance).
      4. Команда из алгоритма (например, "forward", "ascend") влияет на
         горизонтальное и вертикальное положение дрона.
      5. Рисуется scatter-график (xs, ys) — точек лидара относительно дрона,
         который находится в центре (0, 0).
      6. Обновляется окно matplotlib без закрытия (реальный интерактивный режим).
  """
  # Параметры леса
  forest_height = 15.0  # высота леса (м)
  forest_start = 0.0    # передняя граница леса (x=0)
  forest_end = -15.0    # задняя граница леса (x=-15)

  # Начальные параметры дрона
  drone_x = 30.0        # горизонтальное расстояние до леса (м)
  drone_altitude = 10.0 # начальная высота дрона (м)

  # Создаём контроллер (ваш класс с алгоритмом)
  controller = DroneController(initial_altitude=drone_altitude)

  # Настройки симуляции
  simulation_iterations = 200
  horizontal_speed = 1.0  # скорость горизонтального движения (м/итерацию)
  vertical_speed = 0.5    # скорость вертикального движения (м/итерацию)

  # Создаём окно для графика в интерактивном режиме
  plt.ion()
  fig, ax = plt.subplots(figsize=(6, 6))
  plt.show(block=False)

  for iteration in range(simulation_iterations):
    # Текущая высота из контроллера
    drone_altitude = controller.current_altitude

    # 1) Генерация данных лидара: 500 точек
    num_points = 500
    lidar_ranges = []
    angles = []
    for i in range(num_points):
      angle = 2 * math.pi * i / num_points
      angles.append(angle)

      # Определяем, в переднем ли секторе (±60°)
      diff = abs((angle + math.pi) % (2 * math.pi) - math.pi)

      if diff < math.radians(60):
        # Дрон не перелетел лес и находится ниже вершины леса
        if drone_x > forest_start and drone_altitude < forest_height:
          dist = drone_x + random.uniform(-0.2, 0.2)
          lidar_ranges.append(max(0.2, dist))
        # Дрон над лесом (x ∈ [forest_end, forest_start]) и ниже 15 м
        elif (forest_start >= drone_x >= forest_end
              and drone_altitude < forest_height):
          # Имитируем крону — расстояния 5–8 м
          lidar_ranges.append(random.uniform(5, 8))
        else:
          # Иначе открытое пространство
          lidar_ranges.append(random.uniform(15, 20))
      else:
        # Для остальных углов: открытое пространство
        lidar_ranges.append(random.uniform(15, 20))

    # 2) Генерация дальномера вниз (range_down)
    noise_down = random.uniform(-0.1, 0.1)
    if drone_x > forest_start:
      range_down = drone_altitude + noise_down
    elif forest_start >= drone_x >= forest_end:
      if drone_altitude < forest_height:
        # Дрон ниже вершины леса
        range_down = drone_altitude * 0.8 + noise_down
      else:
        # Дрон выше леса
        range_down = (drone_altitude - forest_height) + noise_down
    else:
      # Дрон уже за лесом
      range_down = drone_altitude + noise_down

    # 3) Генерация дальномера вверх (range_up)
    noise_up = random.uniform(-0.1, 0.1)
    if (forest_start >= drone_x >= forest_end
        and drone_altitude < forest_height):
      # Дрон над лесом и ниже вершины
      dist_up = (forest_height - drone_altitude) + noise_up
      range_up = max(0.5, dist_up)
    else:
      # Иначе "небо"
      range_up = 10**9

    # 4) Вызываем алгоритм облёта
    provisional_distance = 1.0
    command = controller.normal_flight_obstacle_avoidance(
        lidar_ranges, [], range_down, range_up, provisional_distance)

    # 5) Определяем, сколько дрон пролетел
    if "forward" in command or "backward" in command:
      distance_flew = horizontal_speed
    elif "ascend" in command or "descend" in command:
      distance_flew = vertical_speed
    else:
      distance_flew = 0

    # 6) Обновляем позицию дрона
    if "forward" in command:
      if drone_x > forest_start:
        drone_x = max(0, drone_x - distance_flew)
      else:
        drone_x -= distance_flew
    elif "backward" in command:
      drone_x += distance_flew
    elif "ascend" in command:
      controller.current_altitude += distance_flew
    elif "descend" in command:
      controller.current_altitude = max(
          0, controller.current_altitude - distance_flew)

    # 7) Визуализация
    xs = [r * math.cos(a) for r, a in zip(lidar_ranges, angles)]
    ys = [r * math.sin(a) for r, a in zip(lidar_ranges, angles)]

    ax.cla()  # Очищаем оси
    ax.scatter(xs, ys, s=5, label="Лидар точки")
    ax.plot(0, 0, 'ro', markersize=8, label="Дрон")

    ax.set_title(
        f"Iter: {iteration+1} | Command: {command} | "
        f"Altitude: {controller.current_altitude:.2f} м")
    ax.set_xlabel("X (м)")
    ax.set_ylabel("Y (м)")
    ax.set_xlim(-25, 25)
    ax.set_ylim(-25, 25)
    ax.set_aspect("equal")
    ax.legend(loc="upper right")

    # Обновляем окно
    fig.canvas.draw()
    fig.canvas.flush_events()

    # 8) Печать информации в консоль
    print(
        f"Iteration {iteration+1}: Command: {command}, "
        f"Altitude: {controller.current_altitude:.2f} м, "
        f"drone_x: {drone_x:.2f} м, range_down: {range_down:.2f}, "
        f"range_up: {range_up if range_up < 10**8 else '10^9'}"
    )

    # Пауза для "анимационного" эффекта
    time.sleep(0.05)

  # По окончании симуляции
  plt.ioff()
  plt.show()


if __name__ == "__main__":
  simulate_forest_scenario()
