import time
import random
import math

from obstacle_avoidance import DroneController


def simulate_forest_scenario():
    """
    Симуляция полёта дрона к лесополосе:
      - лес имеет высоту 15 м и занимает горизонтальную область от x = 0 (начало леса)
        до x = -15 м (конец леса)
      - дрон начинает полёт на высоте 10 м и с горизонтального расстояния 10 м (drone_x = 10),
        то есть лес находится впереди
      - в каждом цикле генерируются данные лидара (500 точек) и показания дальномеров вниз и вверх
      - distance_flew – расстояние, пройденное дроном за итерацию (определяется в зависимости от команды)
      - функция normal_flight_obstacle_avoidance (метод контроллера) вызывается для получения команды,
        после чего обновляется состояние дрона
    """
    # Параметры леса
    forest_height = 15.0  # высота леса (м)
    forest_start = 0.0  # передняя граница леса (лес начинается при x = 0)
    forest_end = -15.0  # задняя граница леса (лес заканчивается при x = -15 м)

    # Начальные параметры дрона
    drone_x = 30.0  # горизонтальное расстояние до леса (положительное – лес впереди)
    drone_altitude = 10.0  # высота дрона (м)

    # Создаём контроллер (здесь используется ваш класс с алгоритмом облёта)
    controller = DroneController(initial_altitude=drone_altitude)

    # Настройки симуляции
    simulation_iterations = 200
    horizontal_speed = 1.0  # скорость горизонтального движения (м/итерацию)
    vertical_speed = 0.5  # скорость вертикального движения (м/итерацию)

    for iteration in range(simulation_iterations):
        # Обновляем текущую высоту из контроллера
        drone_altitude = controller.current_altitude

        # Генерация данных лидара (500 измерений равномерно по окружности)
        num_points = 500
        lidar_ranges = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            # Рассмотрим сектор "вперёд" ±60° (т.е. diff < 60°)
            diff = abs((angle + math.pi) % (2 * math.pi) - math.pi)
            if diff < math.radians(60):
                # Если дрон ещё не перелетел лес (drone_x > forest_start) и ниже вершины леса,
                # то в переднем секторе расстояние = drone_x + небольшой шум
                if drone_x > forest_start and drone_altitude < forest_height:
                    dist = drone_x + random.uniform(-0.2, 0.2)
                    lidar_ranges.append(max(0.2, dist))
                # Если дрон уже находится над лесом, то:
                #  - если он находится над лесом (forest_start >= drone_x >= forest_end) и ниже вершины,
                #    можно имитировать препятствия кроны – например, значения в диапазоне 5–8 м
                elif forest_start >= drone_x >= forest_end and drone_altitude < forest_height:
                    lidar_ranges.append(random.uniform(5, 8))
                else:
                    # Если дрон выше леса или лес уже позади (drone_x < forest_end), считаем, что впереди открытое пространство
                    lidar_ranges.append(random.uniform(15, 20))
            else:
                # Для остальных углов считаем "открытое пространство"
                lidar_ranges.append(random.uniform(15, 20))

        # Генерация показаний дальномера вниз (range_down)
        noise_down = random.uniform(-0.1, 0.1)
        if drone_x > forest_start:
            # Дрон ещё не над лесом – видит землю (расстояние = высота дрона)
            range_down = drone_altitude + noise_down
        elif forest_start >= drone_x >= forest_end:
            # Дрон над лесом
            if drone_altitude < forest_height:
                # Если дрон ниже вершины леса, датчик может "видеть" только крону – пусть значение немного меньше (например, 80% от высоты)
                range_down = drone_altitude * 0.8 + noise_down
            else:
                # Если дрон поднялся выше леса, измеряется разница между высотой дрона и лесом
                range_down = (drone_altitude - forest_height) + noise_down
        else:
            # Дрон уже за лесом – открытое поле: видим землю (расстояние = высота дрона)
            range_down = drone_altitude + noise_down

        # Генерация показаний дальномера вверх (range_up)
        noise_up = random.uniform(-0.1, 0.1)
        if forest_start >= drone_x >= forest_end and drone_altitude < forest_height:
            # Если дрон над лесом и ниже вершины, дальномер вверх показывает расстояние до кроны
            dist_up = (forest_height - drone_altitude) + noise_up
            range_up = max(0.5, dist_up)
        else:
            # В остальных случаях над дроном "только небо" – выдаём очень большое число
            range_up = 10 ** 9

        # Пробное значение для distance_flew (будет скорректировано по команде)
        provisional_distance = 1.0

        # Вызов алгоритма облёта (передаем пустой список для интенсивностей, если он не используется)
        command = controller.normal_flight_obstacle_avoidance(lidar_ranges, [], range_down, range_up,
                                                              provisional_distance)

        # Определяем, сколько дрон пролетел за итерацию, в зависимости от полученной команды
        if "forward" in command or "backward" in command:
            distance_flew = horizontal_speed
        elif "ascend" in command or "descend" in command:
            distance_flew = vertical_speed
        else:
            distance_flew = 0

        # Обновляем положение дрона на основе команды
        if "forward" in command:
            # Если лес еще впереди (drone_x > forest_start), при движении вперед drone_x уменьшается
            if drone_x > forest_start:
                drone_x = max(0, drone_x - distance_flew)
            # Если дрон уже над лесом или перелетел его, drone_x становится отрицательным (лес сзади)
            else:
                drone_x -= distance_flew
        elif "backward" in command:
            drone_x += distance_flew
        elif "ascend" in command:
            controller.current_altitude += distance_flew
        elif "descend" in command:
            controller.current_altitude = max(0, controller.current_altitude - distance_flew)
        # "stop" или иные команды не меняют положение

        # Выводим информацию по итерации
        print(f"Iteration {iteration + 1}: Command: {command}, Altitude: {controller.current_altitude:.2f} м, "
              f"drone_x: {drone_x:.2f} м, range_down: {range_down:.2f}, "
              f"range_up: {range_up if range_up < 10 ** 8 else '10^9'}")

        # Задержка для эмуляции реального времени
        time.sleep(0.2)

if __name__ == "__main__":
    simulate_forest_scenario()
