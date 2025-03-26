import math


class DroneController:
  """Контроллер дрона, реализующий логику обхода препятствий.

  Атрибуты:
    state: str. Текущее состояние дрона (например, "NORMAL", "ASCENDING").
    prev_state: str. Предыдущее состояние дрона, используется для возврата при выходе
        из вспомогательного состояния (EXTREME_ASCENDING и пр.).
    is_extra: bool. Флаг режима "extra" (дополнительное перемещение).
    extra_distance: float. Необходимое расстояние, которое нужно пройти в режиме extra.
    extra_went_distance: float. Накопленное расстояние, пройденное в режиме extra.
    target_altitude: float. Целевая высота дрона.
    current_altitude: float. Текущая высота дрона.
    seen_obstacle: float. Накопленное расстояние в режиме OVERFLYING, если под
        дроном всё ещё есть препятствие.
    went_overflying: float. Накопленное расстояние, пройденное в режиме OVERFLYING.
    vertical_distance_to_obstacle: float. Вертикальное расстояние, нужное
        для безопасного перелёта препятствия.
    seen_obstacle_min_to_overfly: float. Минимальное расстояние, которое дрон
        должен пролететь в режиме OVERFLYING, видя препятствие снизу, чтобы разрешить
        начало спуска.
    is_descending_permited: bool. Флаг, разрешающий спуск (True, когда безопасно).
    min_possible_range_down_to_descned: float. Порог дальномера вниз, при превышении
        которого считаем, что можно безопасно начинать спуск.
    safety_box_half: float. Половина размера защитного бокса (2 м).
    max_lidar_distance: float. Максимальная дистанция, которая считается "опасной"
        для лидара.
    max_rangefinder_distance: float. Максимальная дистанция, которая считается
        "опасной" для дальномера.
    wide_obstacle_angle_threshold: float. Угол (в радианах), определяющий, считается
        ли препятствие широким (например, лесополосой).
    min_collision_points_for_wide: int. Минимальное количество точек столкновения
        лидара, при превышении которого препятствие считается широким.
  """

  def __init__(self, initial_altitude=15):
    """Инициализация контроллера дрона.

    Args:
      initial_altitude: float. Начальная высота дрона.
    """
    self.state = "NORMAL"
    self.prev_state = "NORMAL"

    # EXTRA
    self.is_extra = False
    self.extra_distance = 0
    self.extra_went_distance = 0

    # Target/Current altitude
    self.target_altitude = initial_altitude
    self.current_altitude = initial_altitude

    # OVERFLYING
    self.seen_obstacle = 0
    self.went_overflying = 0
    self.vertical_distance_to_obstacle = 0
    self.seen_obstacle_min_to_overfly = 0.5
    self.is_descending_permited = False
    self.min_possible_range_down_to_descned = 6

    # Параметры системы
    self.safety_box_half = 2.0
    self.max_lidar_distance = 12.0
    self.max_rangefinder_distance = 15.0

    # Параметры для определения широкого препятствия
    self.wide_obstacle_angle_threshold = math.radians(30)
    self.min_collision_points_for_wide = 20

  def set_extra_false(self):
    """Сбрасывает режим extra (дополнительного движения)."""
    self.is_extra = False
    self.extra_distance = 0
    self.extra_went_distance = 0

  def get_collision_distance(self, lidar_ranges, angle=0):
    """Определяет потенциальные столкновения на основе данных лидара.

    Args:
      lidar_ranges: list of float. Расстояния, полученные от лидара.
      angle: float, необязательный. Центральный угол (в рад),
        относительно которого ищем столкновение (по умолчанию 0).

    Returns:
      (collision_angles, min_distance): tuple.
        collision_angles: список углов столкновения (float).
        min_distance: float, минимальное расстояние до препятствия, попадающего
          в защитный бокс.
    """
    num_points = len(lidar_ranges)
    angles = [(i * 2 * math.pi / num_points) for i in range(num_points)]
    collision_angles = []
    min_distance = 10**9

    for i, dist_val in enumerate(lidar_ranges):
      a = angles[i]
      a_diff = a if a <= math.pi else 2 * math.pi - a

      if math.cos(a_diff) > 0:
        lateral_offset = dist_val * math.sin(a_diff)
        if lateral_offset < self.safety_box_half:
          collision_angles.append(a_diff)
          obstacle_distance = math.cos(a_diff) * dist_val
          if obstacle_distance < min_distance:
            min_distance = obstacle_distance
    return collision_angles, min_distance

  def is_wide_obstacle(self, collision_angles):
    """Определяет, является ли препятствие широким (лесополосой).

    Args:
      collision_angles: list of float. Углы столкновения.

    Returns:
      bool: True, если препятствие широкое, иначе False.
    """
    if not collision_angles:
      return False
    sorted_angles = sorted(collision_angles)
    max_segment = 0.0
    current_start = sorted_angles[0]
    current_end = sorted_angles[0]

    for a in sorted_angles[1:]:
      if a - current_end < math.radians(5):
        current_end = a
      else:
        segment = current_end - current_start
        if segment > max_segment:
          max_segment = segment
        current_start = a
        current_end = a

    # Последний сегмент
    segment = current_end - current_start
    if segment > max_segment:
      max_segment = segment

    # Широкое препятствие, если:
    #  - угол непрерывного сегмента >= wide_obstacle_angle_threshold, или
    #  - кол-во точек столкновения >= min_collision_points_for_wide
    if (max_segment >= self.wide_obstacle_angle_threshold or
        len(collision_angles) >= self.min_collision_points_for_wide):
      return True
    return False

  def get_lidar_data(self, lidar_ranges, angle, width):
    """Возвращает расстояние до ближайшего препятствия в заданном секторе.

    Args:
      lidar_ranges: list of float. Измерения лидара (500 точек).
      angle: float. Центральный угол (в рад).
      width: float. Угловая ширина (в рад).

    Returns:
      float: минимальное расстояние до препятствия в заданном секторе.
    """
    n = len(lidar_ranges)
    delta_angle = 2 * math.pi / n
    half_width = width / 2.0
    min_distance = self.max_lidar_distance

    for i, d in enumerate(lidar_ranges):
      theta = i * delta_angle
      diff = abs((theta - angle + math.pi) % (2 * math.pi) - math.pi)
      if diff <= half_width and d < min_distance:
        min_distance = d
    return min_distance

  def action(self, action, distance=-1):
    """Возвращает команду для дрона с учётом поправки на расстояние.

    Args:
      action: str. Исходная команда ("forward", "ascend" и т. д.).
      distance: float, опционально. Расстояние до препятствия.

    Returns:
      str: команда, при необходимости дополненная суффиксом "_slow".
    """
    if action == "error":
      print("ОШИБКА! Функция ничего не вернула.")
      return "stop"

    if distance != -1:
      if action in ("backward", "forward"):
        if distance < self.max_lidar_distance:
          return action + "_slow"
      elif action in ("ascend", "descend"):
        if distance < self.max_rangefinder_distance:
          return action + "_slow"
    return action

  def get_angle_to_goal(self):
    """Заглушка для определения угла на цель."""
    return 0

  def set_state(self, state):
    """Сменить состояние дрона с сохранением предыдущего."""
    if self.state != state:
      self.prev_state = self.state
    self.state = state

  def normal_flight_obstacle_avoidance(self, lidar_ranges, lidar_intensities,
                                       range_down, range_up, distance_flew):
    """Основная функция обработки данных с датчиков для обхода препятствий.

    Логика построена на ряде состояний:
      - NORMAL, ASCENDING, OVERFLYING, DESCENDING, BACKWARDS и др.
      - Учитывается "extra" режим (is_extra), позволяющий выполнять
        дополнительные движения на небольшое расстояние.

    Args:
      lidar_ranges: list of float. Измерения лидара (500 точек).
      lidar_intensities: list of float. Интенсивности лидара (не используется).
      range_down: float. Показание дальномера вниз (м).
      range_up: float. Показание дальномера вверх (м).
      distance_flew: float. Расстояние, пройденное дроном в предыдущем цикле.

    Returns:
      str: Команда управления ("forward", "ascend", "descend", "stop", ...),
           возможно с суффиксом "_slow".
    """
    collision_angles, obstacle_distance = self.get_collision_distance(
        lidar_ranges)
    wide_obstacle = self.is_wide_obstacle(collision_angles)

    # EXTREME_ASCENDING, EXTREME_DESCENDING
    if range_down < self.safety_box_half:
      self.set_state("EXTREME_ASCENDING")
      return self.action("ascend", range_up)

    if range_up < self.safety_box_half:
      self.set_state("EXTREME_DESCENDING")
      return self.action("descend", range_down)

    # Если были в EXTREME, возвращаемся к prev_state
    if self.state in ("EXTREME_DESCENDING", "EXTREME_ASCENDING"):
      self.state = self.prev_state

    # NORMAL / ESCAPE / EXTREME_* ...
    if self.state in ("NORMAL", "EXTREME_DESCENDING", "EXTREME_ASCENDING",
                      "ESCAPE"):
      if obstacle_distance < self.max_lidar_distance:
        if wide_obstacle:
          self.set_state("ASCENDING")
          return self.action("ascend", range_up)
        else:
          # Узкое препятствие (столб)
          left_count = sum(1 for a in collision_angles if math.sin(a) > 0)
          right_count = sum(1 for a in collision_angles if math.sin(a) <= 0)
          self.bypass_direction = "left" if left_count <= right_count else "right"
          self.set_state("SIDEBYPASS")
          self.bypass_distance_went = 0
          return f"side_{self.bypass_direction}"
      else:
        return self.action("forward")

    elif self.state == "ASCENDING":
      if range_up < 4:
        self.set_state("BACKWARDS")
        self.set_extra_false()
        return self.action("backward")
      if obstacle_distance < self.max_lidar_distance:
        self.set_extra_false()
        return self.action("ascend", range_up)
      elif not self.is_extra:
        self.is_extra = True
        self.extra_distance = self.safety_box_half
        return self.action("ascend", range_up)
      elif self.is_extra:
        self.extra_went_distance += distance_flew
        if self.extra_went_distance >= self.safety_box_half:
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
      _, backward_distance = self.get_collision_distance(lidar_ranges, 180)
      if backward_distance < 4:
        self.set_extra_false()
        return self.action("escape")
      if range_up > 4:
        if not self.is_extra:
          self.is_extra = True
          self.extra_distance = self.safety_box_half
          return self.action("backward", backward_distance)
        else:
          self.extra_went_distance += distance_flew
          if self.extra_went_distance >= self.safety_box_half:
            self.set_extra_false()
            self.set_state("ASCENDING")
            return self.action("ascend", range_up)
          else:
            return self.action("backward", backward_distance)
      else:
        self.set_extra_false()
        return self.action("backward", backward_distance)

    elif self.state == "OVERFLYING":
      self.went_overflying += distance_flew
      # Проверяем левый/правый сектор (пример: угол 90°) на препятствия
      # Если есть препятствие, поднимаемся
      if self.get_collision_distance(lidar_ranges, 90)[1] < self.max_lidar_distance:
        self.set_state("ASCENDING")
        self.set_extra_false()
        return self.action("ascend", range_up)

      # Если дрон прошёл над препятствием
      if (range_down < self.vertical_distance_to_obstacle + 4 or
          self.went_overflying > 10):
        self.seen_obstacle += distance_flew

      if (self.seen_obstacle >= self.seen_obstacle_min_to_overfly and
          not self.is_descending_permited):
        self.is_descending_permited = True

      if range_down < self.safety_box_half:
        self.set_state("OVERFLYING")
        self.set_extra_false()
        return self.action("ascend", range_up)
      elif (range_down >= self.min_possible_range_down_to_descned and
            not self.is_extra and self.is_descending_permited):
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
      if range_down < self.safety_box_half:
        self.set_state("ASCENDING")
        return self.action("ascend", range_up)
      elif range_down <= self.target_altitude:
        self.set_state("NORMAL")
        return self.action("forward")
      else:
        return self.action("descend", range_down)

    # По умолчанию
    print("Current state:", self.state)
    return self.action("error")
