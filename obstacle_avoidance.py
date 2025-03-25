import math
import random


class DroneController:
    def __init__(self, initial_altitude=15):
        # Инициализация контроллера дрона.
        # Начальное состояние – NORMAL (обычный полет) на заданной высоте.
        # Возможные состояния: NORMAL, BACKWARSS, ASCENDING, OVERFLYING, DESCENDING, SIDEBYPASS.
        self.state = "NORMAL"
        # Флаг для режима "extra" (дополнительного движения на заданное расстояние)
        self.is_extra = False
        # Расстояние, которое необходимо пройти в режиме extra
        self.extra_distance = 0
        # Накопленное расстояние, пройденное в режиме extra
        self.extra_went_distance = 0
        # Целевая и текущая высота дрона
        self.target_altitude = initial_altitude
        self.current_altitude = initial_altitude
        # Направление обхода препятствия (left/right)
        self.bypass_direction = None
        # Оставшаяся дистанция для обхода препятствия
        self.bypass_distance_remaining = 0.0
        # Пройденное расстояние при обходе препятствия
        self.bypass_distance_went = 0.0
        # Параметры системы:
        self.extra_min_complience = 0.1  # Минимальное расстояние для режима extra (в метрах)
        self.safety_box_half = 2.0  # Половина размера защитного бокса (бокс 4x4x4 м)
        self.max_lidar_distance = 12.0  # Максимальное расстояние, рассматриваемое лиадаром для обнаружения препятствия
        self.max_rangefinder_distance = 15.0  # Максимальное расстояние измерения дальномером
        self.max_bypass_distance_went = 10  # Максимальное расстояние обхода до переключения режима
        # Параметры для определения широкого препятствия (например, лесополоса)
        self.wide_obstacle_angle_threshold = math.radians(30)  # Порог угла (30°) непрерывного сегмента столкновения
        self.min_collision_points_for_wide = 20  # Минимальное количество точек столкновения для классификации препятствия как широкого

    def set_extra_false(self):
        """
        Сброс режима extra.
        Обнуляет флаг is_extra и накопленные дистанции extra.
        """
        self.is_extra = False
        self.extra_distance = 0
        self.extra_went_distance = 0

    def get_collision_distance(self, lidar_ranges, angle=0):
        """
        Обработка данных лидара для определения потенциальных столкновений.

        Параметры:
          lidar_ranges - список расстояний, полученных с лидара (500 измерений по 360°)
          angle - направление, относительно которого производится расчет (по умолчанию 0 радиан, т.е. вперед)

        Алгоритм:
          - Вычисляем для каждого измерения угол, равномерно распределенный по кругу.
          - Определяем абсолютное отклонение от направления движения (a_diff).
          - Если точка находится впереди (cos(a_diff) > 0) и боковое смещение (d*sin(a_diff)) меньше половины защитного бокса,
            фиксируем угол столкновения и вычисляем расстояние по направлению движения (d*cos(a_diff)).
          - Выбираем минимальное из таких расстояний.

        Возвращает:
          collision_angles - список углов столкновения,
          min_distance - минимальное расстояние до препятствия, попадающего в защитный бокс.
        """
        num_points = len(lidar_ranges)
        angles = [(i * 2 * math.pi / num_points) for i in range(num_points)]
        collision_angles = []
        min_distance = 10 ** 9  # Инициализация большим числом для поиска минимума
        for i, d in enumerate(lidar_ranges):
            # Определяем угол измерения
            a = angles[i]
            # Вычисляем минимальное отклонение от направления 0, учитывая симметрию (если угол > π, берем дополнение до 2π)
            a_diff = a if a <= math.pi else 2 * math.pi - a
            # Рассматриваем только точки, находящиеся впереди (cos(a_diff) > 0)
            if math.cos(a_diff) > 0:
                # Вычисляем боковое смещение от линии движения
                lateral_offset = d * math.sin(a_diff)
                if lateral_offset < self.safety_box_half:
                    # Фиксируем угол столкновения
                    collision_angles.append(a_diff)
                    # Вычисляем расстояние до препятствия по направлению движения
                    obstacle_distance = math.cos(a_diff) * d
                    if obstacle_distance < min_distance:
                        min_distance = obstacle_distance
        return collision_angles, min_distance

    def is_wide_obstacle(self, collision_angles):
        """
        Определение, является ли обнаруженное препятствие широким (например, лесополосой).

        Параметры:
          collision_angles - список углов столкновения, полученный из get_collision_distance.

        Алгоритм:
          - Сортируем углы столкновения.
          - Находим максимальный непрерывный сегмент углов, где разрыв между соседними точками не превышает 5°.
          - Если длина сегмента превышает порог (30°) или количество точек столкновения достаточно велико,
            классифицируем препятствие как широкое.

        Возвращает:
          True, если препятствие широкое, иначе False.
        """
        if not collision_angles:
            return False
        sorted_angles = sorted(collision_angles)
        max_segment = 0.0
        current_start = sorted_angles[0]
        current_end = sorted_angles[0]
        # Ищем непрерывный сегмент углов столкновения
        for a in sorted_angles[1:]:
            if a - current_end < math.radians(5):  # Допустимый разрыв до 5° между соседними измерениями
                current_end = a
            else:
                segment = current_end - current_start
                if segment > max_segment:
                    max_segment = segment
                current_start = a
                current_end = a
        # Проверяем последний сегмент
        segment = current_end - current_start
        if segment > max_segment:
            max_segment = segment

        # Если максимальный сегмент больше порогового или число точек столкновения велико, возвращаем True
        if max_segment >= self.wide_obstacle_angle_threshold or len(
                collision_angles) >= self.min_collision_points_for_wide:
            return True
        return False

    def get_lidar_data(self, lidar_ranges, angle, width):
        """
        Получение расстояния до ближайшего препятствия в секторе, заданном центральным углом и шириной сектора.

        Параметры:
          lidar_ranges - список измерений лидара.
          angle - центральный угол сектора (в радианах), в котором ищем препятствие.
          width - угловая ширина сектора (в радианах).

        Алгоритм:
          - Для каждого измерения вычисляем его угол.
          - Определяем разницу между углом измерения и заданным углом, учитывая цикличность углов.
          - Если разница не превышает половину ширины сектора, сравниваем расстояние с текущим минимальным.
          - Если ни одно измерение не попало в сектор, возвращаем максимально допустимое расстояние.

        Возвращает:
          Минимальное расстояние до препятствия в указанном секторе.
        """
        n = len(lidar_ranges)
        delta_angle = 2 * math.pi / n  # Шаг угла между измерениями
        half_width = width / 2.0
        min_distance = self.max_lidar_distance  # Инициализация максимальным значением
        for i, d in enumerate(lidar_ranges):
            # Вычисляем угол текущего измерения
            theta = i * delta_angle
            # Вычисляем разницу между текущим углом и заданным центральным углом (с учетом цикличности)
            diff = abs((theta - angle + math.pi) % (2 * math.pi) - math.pi)
            if diff <= half_width:
                if d < min_distance:
                    min_distance = d
        return min_distance

    def action(self, action, distance=-1):
        """
        Корректирует команду для дрона с учетом расстояния до препятствия.

        Параметры:
          action - базовая команда (например, "forward", "backward", "ascend", "descend").
          distance - измеренное расстояние до препятствия; если не указано, используется значение по умолчанию.

        Алгоритм:
          - Если дистанция меньше максимально допустимой, добавляет суффикс '_slow' для замедленного движения.
          - Если дистанция не задана, возвращает исходную команду.

        Возвращает:
          Скорректированную команду для дрона.
        """
        if distance != -1:
            if action == "backward" or action == "forward":
                if distance < self.max_lidar_distance:
                    return action + "_slow"
            elif action == "ascend" or action == "descend":
                if distance < self.max_rangefinder_distance:
                    return action + "_slow"
        else:
            return action

    def normal_flight_obstacle_avoidance(self, lidar_ranges, lidar_intensities, range_down, range_up, distance_flew):
        """
        Основная функция обработки данных с датчиков для обхода препятствий во время нормального полета.

        Параметры:
          lidar_ranges - список измерений лидара (500 точек по 360°).
          lidar_intensities - список интенсивностей измерений (не используется в логике, передается для полноты).
          range_down - показание дальномера, измеряющего расстояние вниз.
          range_up - показание дальномера, измеряющего расстояние вверх.
          distance_flew - расстояние, пройденное дроном за итерацию (в метрах).

        Алгоритм:
          1. Получаем углы столкновения и минимальное расстояние до препятствия с помощью get_collision_distance.
          2. Определяем, является ли препятствие широким (лесополосой) с помощью is_wide_obstacle.
          3. Если одновременно препятствия сверху и снизу (слишком малые расстояния),
             переходим в режим ESCAPE.
          4. Если препятствие слишком близко снизу или сверху, переключаемся в режим EXTREME_ASCENDING или EXTREME_DESCENDING.
          5. В состоянии NORMAL (и аналогичных) если препятствие обнаружено:
             - При широком препятствии (например, лесополоса) переходим в режим ASCENDING.
             - При узком препятствии (например, столб) определяем сторону обхода и переходим в режим SIDEBYPASS.
          6. В остальных состояниях (ASCENDING, BACKWARDS, OVERFLYING, DESCENDING, SIDEBYPASS)
             реализована соответствующая логика изменения состояния дрона с учетом накопленных extra дистанций.

        Возвращает:
          Команду (и/или направление) для дрона, например: "forward", "ascend", "side_left" и т.п.
        """
        # Получаем углы столкновения и минимальное расстояние до препятствия с помощью лидара
        collision_angles, obstacle_distance = self.get_collision_distance(lidar_ranges)
        # Определяем, является ли препятствие широким (например, лесополосой)
        wide_obstacle = self.is_wide_obstacle(collision_angles)

        # Если одновременно препятствия сверху и снизу (слишком малые значения дальномеров), переключаемся в режим ESCAPE
        if range_down < self.safety_box_half and range_up < self.safety_box_half:
            self.state = "ESCAPE"

        # Если препятствие слишком близко снизу, переключаемся в режим EXTREME_ASCENDING
        if range_down < self.safety_box_half:
            self.state = "EXTREME_ASCENDING"
            return self.action("ascend", range_up)

        # Если препятствие слишком близко сверху, переключаемся в режим EXTREME_DESCENDING
        if range_up < self.safety_box_half:
            self.state = "EXTREME_DESCENDING"
            return self.action("descend", range_down)

        # Обработка в нормальном режиме полета и похожих состояниях (NORMAL, EXTREME_*, ESCAPE)
        if self.state == "NORMAL" or self.state == "EXTREME_DESCENDING" or self.state == "EXTREME_ASCENDING" or self.state == "ESCAPE":
            # Если обнаружено препятствие перед дроном (расстояние меньше максимально допустимого)
            if obstacle_distance < self.max_lidar_distance:
                if wide_obstacle:
                    # Если препятствие широкое, переходим в режим ASCENDING (подъем)
                    self.state = "ASCENDING"
                    return self.action("ascend", range_up)
                else:
                    # Если препятствие узкое (например, столб)
                    # Считаем количество точек столкновения с левой и правой стороны
                    left_count = sum(1 for a in collision_angles if math.sin(a) > 0)
                    right_count = sum(1 for a in collision_angles if math.sin(a) <= 0)
                    # Выбираем сторону обхода с меньшим числом столкновений
                    self.bypass_direction = "left" if left_count <= right_count else "right"
                    # Переключаемся в режим обхода сбоку (SIDEBYPASS)
                    self.state = "SIDEBYPASS"
                    self.bypass_distance_went = 0
                    # Возвращаем команду поворота в сторону обхода
                    return f"side_{self.bypass_direction}"
            else:
                # Если препятствий нет, продолжаем прямой полет
                return self.action("forward")

        elif self.state == "ASCENDING":
            # Если расстояние до препятствия сверху слишком мало, переключаемся в режим BACKWARDS
            if range_up < 4:
                self.state = "BACKWARDS"
                self.set_extra_false()
                return self.action("backward")
            # Если препятствие перед дроном всё ещё обнаруживается, продолжаем подъем
            if obstacle_distance < self.max_lidar_distance:
                self.set_extra_false()
                return self.action("ascend", range_up)
            # Если путь вперед свободен, активируем режим extra для дополнительного подъема
            elif not self.is_extra:
                self.is_extra = True
                self.extra_distance = self.safety_box_half
                return self.action("ascend", range_up)
            elif self.is_extra:
                # Накопление extra пройденного расстояния
                self.extra_went_distance += distance_flew
                if self.extra_went_distance >= self.safety_box_half:
                    # Если накоплено нужное extra расстояние, завершаем режим extra и переходим в OVERFLYING
                    self.set_extra_false()
                    self.state = "OVERFLYING"
                    return self.action("forward")
                else:
                    return self.action("ascend", range_up)

        elif self.state == "BACKWARDS":
            # Получаем расстояние до препятствия позади (на 180°)
            _, backward_distance = self.get_collision_distance(lidar_ranges, 180)
            if backward_distance < 4:
                # Если препятствие слишком близко сзади, выходим в режим ESCAPE
                self.set_extra_false()
                return self.action("escape")
            if range_up > 4:
                if not self.is_extra:
                    # Если путь вперед свободен, активируем режим extra для движения назад
                    self.is_extra = True
                    self.extra_distance = self.safety_box_half
                    return self.action("backward", backward_distance)
                elif self.is_extra:
                    self.extra_went_distance += distance_flew
                    if self.extra_went_distance >= self.safety_box_half:
                        # Если extra расстояние достигнуто, переключаемся обратно в ASCENDING
                        self.set_extra_false()
                        self.state = "ASCENDING"
                        return self.action("ascend", range_up)
                    else:
                        return self.action("backward", backward_distance)
            else:
                self.set_extra_false()
                return self.action("backward", backward_distance)

        elif self.state == "OVERFLYING":
            # В режиме OVERFLYING перелетаем препятствие на повышенной высоте
            # Если препятствие впереди обнаружено (наличие точек столкновения), переходим в ASCENDING
            if collision_angles:
                self.state = "ASCENDING"
                self.set_extra_false()
                return self.action("ascend", range_up)
            # Если препятствие под дроном обнаружено, остаемся в OVERFLYING
            if range_down < self.safety_box_half:
                self.state = "OVERFLYING"
                self.set_extra_false()
                return self.action("ascend", range_up)
            elif range_down > 6 and not self.is_extra:
                # Если путь вниз свободен и режим extra не активен, активируем extra
                self.is_extra = True
                self.extra_distance = self.safety_box_half
                return self.action("forward")
            elif self.is_extra:
                if range_down < 6:
                    self.set_extra_false()
                    return self.action("forward")
                self.extra_went_distance += distance_flew
                if self.extra_went_distance >= self.extra_min_complience:
                    self.set_extra_false()
                    self.state = "DESCENDING"
                    return self.action("descend", range_down)
                else:
                    return self.action("forward")

        elif self.state == "DESCENDING":
            # При снижении: если препятствие снизу слишком близко, переключаемся в ASCENDING,
            # если расстояние снижения <= 6 м, переходим в NORMAL, иначе продолжаем снижаться.
            if range_down < self.safety_box_half:
                self.state = "ASCENDING"
                return self.action("ascend", range_up)
            elif range_down <= 6:
                self.state = "NORMAL"
                return self.action("forward")
            else:
                return self.action("descend", range_down)

        elif self.state == "SIDEBYPASS":
            # Режим обхода сбоку: накапливаем пройденное расстояние обхода
            self.bypass_distance_went += distance_flew
            # Если в процессе обхода обнаружено препятствие (расстояние меньше max_lidar_distance)
            if self.get_collision_distance(lidar_ranges)[1] < self.max_lidar_distance:
                if self.bypass_distance_went >= self.max_bypass_distance_went:
                    # Если пройденное расстояние превышает максимум, переключаемся в ASCENDING
                    self.state = "ASCENDING"
                    return self.action("ascend", range_up)
                else:
                    return self.action(f"side_{self.bypass_direction}")
            elif not self.is_extra:
                # Если режим extra не активирован, включаем его
                self.is_extra = True
                self.extra_distance = self.safety_box_half
                return self.action(f"side_{self.bypass_direction}")
            elif self.is_extra:
                if self.get_collision_distance(lidar_ranges)[1] < self.max_lidar_distance:
                    self.set_extra_false()
                    return self.action(f"side_{self.bypass_direction}")
                self.extra_went_distance += distance_flew
                if self.extra_went_distance >= self.extra_min_complience:
                    self.set_extra_false()
                    self.state = "SIDEBYPASS_THROUGH"
                    return self.action("forward", self.get_collision_distance(lidar_ranges)[1])
                else:
                    return self.action("forward")

        elif self.state == "SIDEBYPASS_THROUGH":
            # Состояние SIDEBYPASS_THROUGH пока не реализовано.
            pass

        # По умолчанию возвращаем команду forward
        return self.action("forward")


# Пример использования функции в цикле (симуляция):
if __name__ == "__main__":
    controller = DroneController(initial_altitude=15)

    # Симуляция 10 итераций цикла полета
    for iteration in range(10):
        # Генерируем случайные данные лидара: 500 измерений с расстояниями от 1 до 10 м
        lidar_ranges = [random.uniform(1, 10) for _ in range(500)]
        # Генерируем случайные интенсивности для лидара (не используются в логике)
        lidar_intensities = [random.uniform(0, 1) for _ in range(500)]
        # Симулируем показания дальномеров: вниз и вверх (в метрах)
        range_down = random.uniform(0.5, 12)
        range_up = random.uniform(0.5, 12)
        print("range_down:", str(range_down) + ", range_up:", range_up)
        # Вызываем функцию обхода препятствий с текущими данными
        what_have_done = controller.normal_flight_obstacle_avoidance(
            lidar_ranges, lidar_intensities, range_down, range_up, distance_flew=random.randint(10, 50) / 100
        )
        print(
            f"Итерация {iteration + 1}: Действие: {what_have_done}, Текущая высота: {controller.current_altitude:.2f} м, Состояние: {controller.state}")
