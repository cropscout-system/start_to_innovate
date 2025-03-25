import math
import random


class DroneController:
    def __init__(self, initial_altitude=15):
        # Инициализация контроллера дрона.
        # Начальное состояние – NORMAL (обычный полет) на заданной высоте.
        # Возможные состояния: NORMAL, BACKWARDS, ASCENDING, OVERFLYING, DESCENDING, SIDEBYPASS.
        self.state = "NORMAL"
        self.prev_state = "NORMAL"
        # EXTRA:
        self.is_extra = False
        self.extra_distance = 0
        self.extra_went_distance = 0
        # Целевая и текущая высота дрона
        self.target_altitude = initial_altitude
        self.current_altitude = initial_altitude
        # SIDEBYPASS:
        self.bypass_direction = None
        self.bypass_distance_remaining = 0.0
        self.bypass_distance_went = 0.0
        self.max_bypass_distance_went = 4
        self.sidebypass_extra_distance = 0.5
        # SYDEBYPASS_THROUGH:
        self.max_sidebypass_through_went = 4
        self.bypass_through_went = 0
        # OVERFLYING:
        self.seen_obstacle = 0
        self.went_overflying = 0
        self.vertical_distance_to_obstacle = 0
        self.seen_obstacle_min_to_overfly = 0.5
        self.is_descending_permited = False
        self.min_possible_range_down_to_descned = 6
        # Параметры системы:
        self.safety_box_half = 2.0
        self.max_lidar_distance = 12.0
        self.max_rangefinder_distance = 15.0
        # Параметры для определения широкого препятствия
        self.wide_obstacle_angle_threshold = math.radians(30)
        self.min_collision_points_for_wide = 20

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
        """
        num_points = len(lidar_ranges)
        angles = [(i * 2 * math.pi / num_points) for i in range(num_points)]
        collision_angles = []
        min_distance = 10 ** 9  # Инициализация большим числом для поиска минимума
        for i, d in enumerate(lidar_ranges):
            a = angles[i]
            a_diff = a if a <= math.pi else 2 * math.pi - a

            if math.cos(a_diff) > 0:
                lateral_offset = d * math.sin(a_diff)
                if lateral_offset < self.safety_box_half:
                    collision_angles.append(a_diff)
                    obstacle_distance = math.cos(a_diff) * d
                    if obstacle_distance < min_distance:
                        min_distance = obstacle_distance
        return collision_angles, min_distance


    def is_wide_obstacle(self, collision_angles):
        """
        Определение, является ли обнаруженное препятствие широким (например, лесополосой).
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
        """
        if action == "error":
            print("ОШИБКА! Функция ничего не вернула.")
            return "stop"
        if distance != -1:
            if action == "backward" or action == "forward":
                if distance < self.max_lidar_distance:
                    return action + "_slow"
            elif action == "ascend" or action == "descend":
                if distance < self.max_rangefinder_distance:
                    return action + "_slow"

        return action

    def get_angle_to_goal(self):
        # Необходимо реализовать
        return 0
    
    def set_state(self, state):
        if self.state != state:
            self.prev_state = self.state
        self.state = state

    def normal_flight_obstacle_avoidance(self, lidar_ranges, lidar_intensities, range_down, range_up, distance_flew):
        """
        Основная функция обработки данных с датчиков для обхода препятствий во время нормального полета.
        """
        # Получаем углы столкновения и минимальное расстояние до препятствия с помощью лидара

        collision_angles, obstacle_distance = self.get_collision_distance(lidar_ranges)
        # Определяем, является ли препятствие широким (например, лесополосой)
        wide_obstacle = self.is_wide_obstacle(collision_angles)

        #print("State:", self.state, " Extra:", self.is_extra, "Distance_forward:", self.get_collision_distance(lidar_ranges)[1], "seen_obstacle:", self.seen_obstacle)

        # Если одновременно препятствия сверху и снизу (слишком малые значения дальномеров), переключаемся в режим ESCAPE
        if range_down < self.safety_box_half and range_up < self.safety_box_half:
            self.set_state("ESCAPE")

        # Если препятствие слишком близко снизу, переключаемся в режим EXTREME_ASCENDING
        if range_down < self.safety_box_half:
            self.set_state("EXTREME_ASCENDING")
            return self.action("ascend", range_up)

        # Если препятствие слишком близко сверху, переключаемся в режим EXTREME_DESCENDING
        if range_up < self.safety_box_half:
            self.set_state("EXTREME_DESCENDING")
            return self.action("descend", range_down)

        if self.state == "EXTREME_DESCENDING" or self.state == "EXTREME_ASCENDING":
            self.state = self.prev_state

        # Обработка в нормальном режиме полета и похожих состояниях (NORMAL, EXTREME_*, ESCAPE)
        if self.state == "NORMAL" or self.state == "EXTREME_DESCENDING" or self.state == "EXTREME_ASCENDING" or self.state == "ESCAPE":
            # Если обнаружено препятствие перед дроном (расстояние меньше максимально допустимого)
            if obstacle_distance < self.max_lidar_distance:
                if wide_obstacle:
                    # Если препятствие широкое, переходим в режим ASCENDING (подъем)
                    self.set_state("ASCENDING")
                    return self.action("ascend", range_up)
                else:
                    # Если препятствие узкое (например, столб)
                    # Считаем количество точек столкновения с левой и правой стороны
                    left_count = sum(1 for a in collision_angles if math.sin(a) > 0)
                    right_count = sum(1 for a in collision_angles if math.sin(a) <= 0)
                    # Выбираем сторону обхода с меньшим числом столкновений
                    self.bypass_direction = "left" if left_count <= right_count else "right"
                    # Переключаемся в режим обхода сбоку (SIDEBYPASS)
                    self.set_state("SIDEBYPASS")
                    self.bypass_distance_went = 0
                    # Возвращаем команду поворота в сторону обхода
                    return f"side_{self.bypass_direction}"
            else:
                # Если препятствий нет, продолжаем прямой полет
                return self.action("forward")

        elif self.state == "ASCENDING":
            # Если расстояние до препятствия сверху слишком мало, переключаемся в режим BACKWARDS
            if range_up < 4:
                self.set_state("BACKWARDS")
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
                    self.vertical_distance_to_obstacle = self.extra_went_distance
                    self.set_extra_false()
                    self.set_state("OVERFLYING")
                    self.seen_obstacle = 0
                    self.went_overflying = 0
                    self.is_descending_permited = False
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
                        self.set_state("ASCENDING")
                        return self.action("ascend", range_up)
                    else:
                        return self.action("backward", backward_distance)
            else:
                self.set_extra_false()
                return self.action("backward", backward_distance)

        elif self.state == "OVERFLYING":
            # В режиме OVERFLYING перелетаем препятствие на повышенной высоте
            # Если препятствие впереди обнаружено (наличие точек столкновения), переходим в ASCENDING
            self.went_overflying += distance_flew
            if self.get_collision_distance(lidar_ranges, 90)[1] < self.max_lidar_distance:
                self.set_state("ASCENDING")
                self.set_extra_false()
                return self.action("ascend", range_up)
            if range_down < self.vertical_distance_to_obstacle + 4 or self.went_overflying > 10:
                self.seen_obstacle += distance_flew
            if self.seen_obstacle >= self.seen_obstacle_min_to_overfly and not self.is_descending_permited:
                self.is_descending_permited = True
            # Если препятствие под дроном обнаружено, остаемся в OVERFLYING
            if range_down < self.safety_box_half:
                self.set_state("OVERFLYING")
                self.set_extra_false()
                return self.action("ascend", range_up)
            elif range_down >= self.min_possible_range_down_to_descned and not self.is_extra and self.is_descending_permited:
                # Если путь вниз свободен и режим extra не активен, активируем extra
                self.is_extra = True
                self.extra_distance = self.safety_box_half
                return self.action("forward")
            elif self.is_extra:
                if range_down < self.min_possible_range_down_to_descned:
                    self.set_extra_false()
                    return self.action("forward")
                self.extra_went_distance += distance_flew
                if self.extra_went_distance >= self.safety_box_half:
                    self.set_extra_false()
                    self.set_state("DESCENDING")
                    return self.action("descend", range_down)
                else:
                    return self.action("forward")
            else:
                return self.action("forward")

        elif self.state == "DESCENDING":
            # При снижении: если препятствие снизу слишком близко, переключаемся в ASCENDING,
            # если расстояние снижения <= 6 м, переходим в NORMAL, иначе продолжаем снижаться.
            if range_down < self.safety_box_half:
                self.set_state("ASCENDING")
                return self.action("ascend", range_up)
            elif range_down <= self.target_altitude:
                self.set_state("NORMAL")
                return self.action("forward")
            else:
                return self.action("descend", range_down)

        elif self.state == "SIDEBYPASS":
            if self.bypass_direction == "right" and self.get_collision_distance(lidar_ranges, 90)[1] < self.max_lidar_distance:
                self.set_state("ASCENDING")
                return self.action("turn_to_goal")
            if self.bypass_direction == "left" and self.get_collision_distance(lidar_ranges, -90)[1] < self.max_lidar_distance:
                self.set_state("ASCENDING")
                return self.action("turn_to_goal")
            # Режим обхода сбоку: накапливаем пройденное расстояние обхода
            self.bypass_distance_went += distance_flew
            # Если в процессе обхода обнаружено препятствие (расстояние меньше max_lidar_distance)
            if self.get_collision_distance(lidar_ranges)[1] < self.max_lidar_distance:
                if self.bypass_distance_went >= self.max_bypass_distance_went:
                    # Если пройденное расстояние превышает максимум, переключаемся в ASCENDING
                    self.set_state("ASCENDING")
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
                if self.extra_went_distance >= self.sidebypass_extra_distance:
                    self.set_extra_false()
                    self.set_state("SIDEBYPASS_THROUGH")
                    return self.action("forward", self.get_collision_distance(lidar_ranges)[1])
                else:
                    return self.action("forward")

        elif self.state == "SIDEBYPASS_THROUGH":
            if self.get_collision_distance(lidar_ranges)[1] < self.max_lidar_distance:
                self.set_state("ASCENDING")
                return self.action("ascend", range_up)
            if range_down < self.safety_box_half:
                self.set_state("OVERFLYING")
                self.set_extra_false()
                return self.action("ascend", range_up)
            elif self.get_collision_distance(self, lidar_ranges, self.get_angle_to_goal())[1] >= self.max_lidar_distance:
                self.set_state("NORMAL")
                return self.action("turn_to_goal")
            else:
                self.bypass_through_went += distance_flew
                if self.bypass_through_went >= self.max_sidebypass_through_went:
                    # Если пройденное расстояние превышает максимум, переключаемся в ASCENDING
                    self.set_state("ASCENDING")
                    return self.action("turn_to_goal")
                else:
                    return self.action("forward")

        # По умолчанию возвращаем команду forward
        print("Current state:", self.state)
        return self.action("error")


# Предполагается, что класс DroneController уже импортирован и содержит метод normal_flight_obstacle_avoidance
# (с параметрами: lidar_ranges, lidar_intensities, range_down, range_up, distance_flew)

