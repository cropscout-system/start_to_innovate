import math
import time

import rclpy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import (
    GlobalPositionTarget,
    HomePosition,
    PositionTarget,
    State,
    StatusText,
)
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from pygeodesy.geoids import GeoidPGM
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, NavSatFix, Range
from std_msgs.msg import Bool, Float32, Float64, Header, Int32, String

from .log_file import LogFile

SITL = False
DEBUG = False


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


class FlightPhase:
    def __init__(self, name=None, status=None, point=None):
        self.name = name
        self.status = status
        self.point = point

    def __repr__(self):
        return f"Name = {self.name}, Status = {self.status}, Point = {self.point}"


class SystemChecker:
    def __init__(self):
        self.state = False
        self.battery = False
        self.home_position = True
        self.GPS = False
        self.rangefinder = False
        self.OK = False

    def self_check(self):
        if (
            self.state
            and self.battery
            and self.home_position
            and self.GPS
            and self.rangefinder
        ):
            self.OK = True
            return True
        print(
            f"INIT State = {self.state}, Battery = {self.battery}, Home = {self.home_position}, GPS = {self.GPS}, RF = {self.rangefinder}"
        )
        return False


class TFL(Node):
    def __init__(self):
        super().__init__("tfl_node")
        self.log_file = LogFile()

        # Initialize variables
        self.current_flight_phase = FlightPhase(name="initializing")
        self.current_state = State()
        self.battery_state = BatteryState()
        self.current_global_position = NavSatFix()  # Current global position
        self.rangefinder = Range()
        self.home_position = HomePosition()
        self.sys_messages = StatusText()

        self.system_check = SystemChecker()

        self.current_altitude_home = 0.0  # Relative altitude in meters
        self.current_altitude_amsl = None
        self.current_altitude_rangefinder = None

        self.takeoff_altitude = 2.0
        self.takeoff_altitude_reached = False

        self.target_photo_altitude = 1.0
        self.target_photo_altitude_reached = False

        self.intermediate_altitude = 2.0
        self.intermediate_altitude_reached = False

        self.target_point_altitude = 2.0  # point altitude which we will fly

        self.geo_points = {
            1: GeoPoint(latitude=55.5565773, longitude=37.4197638, altitude=0.0),
            2: GeoPoint(latitude=55.5566212, longitude=37.4193537, altitude=0.0),
        }
        self.intermediate_coordinates = GeoPoint(
            latitude=0.0, longitude=0.0, altitude=0.0
        )

        self._egm96 = GeoidPGM("/usr/share/GeographicLib/geoids/egm96-5.pgm", kind=-3)

        self.position_adjustment_timestamp = (
            None  # время начала поиска посадочной метки
        )
        self.position_adjustment_delay = 30.0  # задержка

        # Define a QoS profile with BEST_EFFORT reliability
        qos_best_effort_profile = QoSProfile(
            depth=10,  # Queue size
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match the publisher's reliability
        )

        # Subscribers
        self.state_sub = self.create_subscription(
            State, "/mavros/state", self.state_callback, 10
        )
        self.battery_sub = self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.battery_callback,
            qos_best_effort_profile,
        )
        self.rangefinder_sub = self.create_subscription(
            Range, "/mavros/rangefinder/rangefinder", self.rangefinder_callback, 10
        )
        self.relative_altitude_sub = self.create_subscription(
            Float64,
            "/mavros/global_position/rel_alt",
            self.relative_altitude_callback,
            qos_best_effort_profile,
        )
        self.global_position_sub = self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.global_position_callback,
            qos_best_effort_profile,
        )
        self.home_position_sub = self.create_subscription(
            HomePosition, "/mavros/home_position/home", self.home_position_callback, 10
        )
        self.sys_messages_sub = self.create_subscription(
            StatusText,
            "/mavros/statustext/recv",
            self.sys_messages_callback,
            qos_best_effort_profile,
        )
        self.speed_position_aruco_sub = self.create_subscription(
            TwistStamped, "/drone/aruco/speed_position", self.speed_position_aruco, 10
        )
        self.photo_taken_sub = self.create_subscription(
            String, "/drone/photo/take_photo/status", self.photo_taken_callback, 1
        )

        # Publishers
        self.landing_procedure = self.create_publisher(Bool, "/drone/land", 10)
        self.landing_procedure.publish(Bool(data=False))
        self.setpoint_raw_global_pub = self.create_publisher(
            GlobalPositionTarget, "/mavros/setpoint_raw/global", 10
        )
        self.setpoint_raw_local_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )
        self.speed_pub = self.create_publisher(Float32, "/drone/speed", 1)
        self.take_photo_pub = self.create_publisher(Int32, "/drone/photo/take_photo", 1)

        # Timers
        self.altitude_observer_timer = self.create_timer(1, self.altitude_observer)
        self.flight_phases_timer = self.create_timer(5, self.flight_phases_observer)
        self.log_data_timer = self.create_timer(1, self.log_data_callback)

        if SITL:
            self.SITL_speed_correction_timer = self.create_timer(
                0.2, self.SITL_speed_correction
            )
        else:
            self.altitude_correction_by_rangefinder_timer = self.create_timer(
                0.2, self.altitude_correction_by_rangefinder
            )

        self.check_position_timer = self.create_timer(2, self.check_position_reached)
        self.system_check_timer = self.create_timer(2, self.system_check.self_check)

        # Services
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, "/mavros/cmd/land")

        # Wait for services to be available
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_mode service...")
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for takeoff service...")
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for land service...")

    def state_callback(self, msg):
        self.current_state = msg
        if msg and msg.system_status in (
            3,
            4,
        ):  # MAV_STATE_STANDBY = 3, MAV_STATE_ACTIVE = 4
            self.system_check.state = True
            if DEBUG:
                self.get_logger().info(
                    f"OK - System check level = {self.current_state.system_status}"
                )

    def relative_altitude_callback(self, msg):
        self.current_altitude_home = (
            msg.data
        )  # Extract the altitude value from the Float64 message

    def global_position_callback(self, msg):
        self.current_global_position = msg
        self.current_altitude_amsl = msg.altitude
        if msg and msg.latitude != 0.0:
            self.system_check.GPS = True
            if DEBUG:
                self.get_logger().info(
                    f"OK - System global position alt = {self.current_global_position.altitude:.1f}"
                )

    def battery_callback(self, msg):
        self.battery_state = msg
        if msg and msg.percentage >= 0.3:
            self.system_check.battery = True
            if DEBUG:
                self.get_logger().info(
                    f"OK - Battery level = {self.battery_state.percentage:.1f}"
                )

    def rangefinder_callback(self, msg):
        self.rangefinder = msg
        if msg and msg.range and msg.range >= 0.1:
            self.system_check.rangefinder = True
            self.current_altitude_rangefinder = msg.range
            if DEBUG:
                self.get_logger().info(
                    f"OK - Rangefinder = {self.current_altitude_rangefinder:.2f}"
                )

    def sys_messages_callback(self, msg):
        self.sys_messages = msg
        self.get_logger().info(f"MESSAGE {msg.severity}-{msg.text}")

    def photo_taken_callback(self, msg: String):
        self.get_logger().info(
            f"Photo callback {msg.data}, {self.current_flight_phase.name}, {self.current_flight_phase.status}"
        )
        if (
            self.current_flight_phase.name == "taking_photo"
            and not self.current_flight_phase.status
        ) or (
            self.current_flight_phase.name == "descending_to_photo_altitude"
            and self.current_flight_phase.status == "reached"
        ):
            self.get_logger().info(f"Photo status {msg.data}")
            self.current_flight_phase = FlightPhase(
                name="taking_photo",
                status="reached",
                point=self.current_geo_point_processing,
            )

    def home_position_callback(self, msg):
        self.home_position = msg
        self.geo_points[1].altitude = msg.geo.altitude + self.target_point_altitude
        self.geo_points[2].altitude = msg.geo.altitude + self.target_point_altitude
        if msg.geo.latitude != 0.0:
            self.system_check.home_position = True
            if DEBUG:
                self.get_logger().info(
                    f"OK - Home position fixed = {self.home_position.geo.latitude:.3f},{self.home_position.geo.longitude:.3f}, {self.home_position.geo.altitude:.2f}"
                )

    def log_data_callback(self):
        log = f"{self.current_flight_phase.name},{self.current_flight_phase.status},{self.current_state.armed},{self.current_state.mode},{self.current_global_position.latitude},{self.current_global_position.longitude},{self.current_altitude_home},{self.current_global_position.altitude},{self.current_altitude_rangefinder},{self.battery_state.current},{self.battery_state.percentage},{self.battery_state.voltage},{self.home_position.geo.latitude},{self.home_position.geo.longitude},{self.home_position.geo.altitude},{self.sys_messages.severity},{self.sys_messages.text}"
        self.log_file.append_string(log)

    def land_drone(self):
        self.get_logger().info("Landing drone...")
        req = CommandTOL.Request()
        req.altitude = 0.0  # Land at current position
        req.latitude = 0.0  # Latitude (not used in most cases)
        req.longitude = 0.0  # Longitude (not used in most cases)
        req.min_pitch = 0.0  # Minimum pitch angle (optional)
        req.yaw = 0.0  # Yaw angle (optional)

        # Call the land service
        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future is not None and future.result().success:
            self.get_logger().info("Landing command sent successfully")
        else:
            self.get_logger().error("Failed to send landing command")

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        if (
            future is not None
            and future.result() is not None
            and future.result().mode_sent
        ):
            self.get_logger().info(f"{mode} mode set successfully")
            return True
        self.get_logger().error("Failed to set mode")
        return False

    def arm_drone(self):
        self.get_logger().info("Arming drone...")
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        self.current_flight_phase = FlightPhase(name="arming")
        rclpy.spin_until_future_complete(self, future)
        if future is not None and future.result().success:
            self.get_logger().info("Drone armed successfully")
            self.current_flight_phase.status = "reached"
            return True
        self.get_logger().error("Failed to arm drone")
        return False

    def takeoff(self):
        self.get_logger().info(f"Taking off to {self.takeoff_altitude} meters...")

        # Create a takeoff request
        req = CommandTOL.Request()
        req.altitude = self.takeoff_altitude  # Set the desired altitude
        req.latitude = 0.0  # Latitude (not used in most cases)
        req.longitude = 0.0  # Longitude (not used in most cases)
        req.min_pitch = 0.0  # Minimum pitch angle (optional)
        req.yaw = 0.0  # Yaw angle (optional)

        # Call the takeoff service
        future = self.takeoff_client.call_async(req)
        self.current_flight_phase = FlightPhase(name="takeoff")

        rclpy.spin_until_future_complete(self, future)
        if future is not None and future.result().success:
            self.get_logger().info("Takeoff command sent successfully")
            return True
        self.get_logger().error("Failed to send takeoff command")
        return False

    def altitude_observer(self):
        if self.current_state.armed and self.current_state.mode == "GUIDED":
            # Высота взлёта достигнута
            if (
                self.current_flight_phase.name == "takeoff"
                and not self.current_flight_phase.status
                and abs(self.current_altitude_home - self.takeoff_altitude) <= 0.2
            ):
                self.takeoff_altitude_reached = True
                self.current_flight_phase.status = "reached"
                self.get_logger().info("Takeoff altitude reached!")

    def flight_phases_observer(self):
        horiz_speed = Float32(data=3.0)

        if self.current_state.armed and self.current_state.mode == "GUIDED":
            # Высота взлёта достигнута

            self.speed_pub.publish(horiz_speed)
            time.sleep(0.1)

            if (
                self.current_flight_phase.name == "takeoff"
                and self.current_flight_phase.status == "reached"
            ):
                self.publish_local_position(0, 0, 0)
                time.sleep(0.1)

                self.current_geo_point_processing = 1
                self.current_flight_phase = FlightPhase(
                    name="move_to_intermediate", point=self.current_geo_point_processing
                )
                self.calculate_intermediate_coordinates(
                    point_id=self.current_geo_point_processing
                )
                self.publish_global_position(
                    self.intermediate_coordinates.latitude,
                    self.intermediate_coordinates.longitude,
                    self.intermediate_coordinates.altitude,
                )

                self.get_logger().info(
                    f"Moving FROM HERE {self.current_global_position.latitude}, {self.current_global_position.longitude}, {self.current_global_position.altitude}"
                )
                self.get_logger().info(
                    f"Moving TO {self.intermediate_coordinates.latitude}, {self.intermediate_coordinates.longitude}, {self.intermediate_coordinates.altitude}"
                )

            # движение к промежуточной точке завершено
            if (
                self.current_flight_phase.name == "move_to_intermediate"
                and self.current_flight_phase.status == "reached"
            ):
                self.current_flight_phase = FlightPhase(
                    name="move_towards_point", point=self.current_geo_point_processing
                )

                self.publish_global_position(
                    self.geo_points.get(self.current_geo_point_processing).latitude,
                    self.geo_points.get(self.current_geo_point_processing).longitude,
                    self.geo_points.get(self.current_geo_point_processing).altitude,
                )
                self.get_logger().info(
                    f"Moving toward point {self.current_geo_point_processing}"
                )

            # движение к целевой точке завершено
            if (
                self.current_flight_phase.name == "move_towards_point"
                and self.current_flight_phase.status == "reached"
            ):
                self.current_flight_phase = FlightPhase(
                    name="descending_to_photo_altitude",
                    point=self.current_geo_point_processing,
                )

                self.get_logger().info(
                    f"Descending to photo altitude at geo point {self.current_geo_point_processing}"
                )

            # Снижение для фото завершено
            if (
                self.current_flight_phase.name == "descending_to_photo_altitude"
                and self.current_flight_phase.status == "reached"
            ):
                self.current_flight_phase = FlightPhase(
                    name="taking_photo", point=self.current_geo_point_processing
                )
                self.get_logger().info(
                    f"Descending to photo altitude at geo point {self.current_geo_point_processing} completed"
                )

            # Съёмка фото
            if (
                self.current_flight_phase.name == "taking_photo"
                and self.current_flight_phase.status == "reached"
            ):
                self.current_flight_phase = FlightPhase(
                    name="ascending_from_photo_altitude"
                )
                self.get_logger().info("Photo Job Done")

            # подъём с фотосъёмки завершён
            if (
                self.current_flight_phase.name == "ascending_from_photo_altitude"
                and self.current_flight_phase.status == "reached"
            ):
                if (
                    len(self.geo_points) > self.current_geo_point_processing
                ):  # точки не закончились - летим в промежуточную
                    self.current_geo_point_processing += 1
                    self.current_flight_phase = FlightPhase(
                        name="move_to_intermediate",
                        point=self.current_geo_point_processing,
                    )
                    self.calculate_intermediate_coordinates(
                        point_id=self.current_geo_point_processing
                    )
                    self.publish_global_position(
                        self.intermediate_coordinates.latitude,
                        self.intermediate_coordinates.longitude,
                        self.intermediate_coordinates.altitude,
                    )
                    self.get_logger().info("Moving to intermediate point ")

                else:
                    self.get_logger().info("No points, Go HOME!")
                    self.current_flight_phase = FlightPhase(name="return_to_home")
                    self.publish_global_position(
                        self.home_position.geo.latitude,
                        self.home_position.geo.longitude,
                        self.home_position.geo.altitude + self.target_point_altitude,
                    )

            if (
                self.current_flight_phase.name == "return_to_home"
                and self.current_flight_phase.status == "reached"
            ):
                self.get_logger().info("Adjustment procedure")
                self.current_flight_phase = FlightPhase(
                    name="landing_position_adjustment"
                )

            if (
                self.current_flight_phase.name == "landing_position_adjustment"
                and self.current_flight_phase.status == "reached"
            ):
                self.get_logger().info("Precious Landing procedure started")
                self.current_flight_phase = FlightPhase(name="precious_landing")

            if (
                self.current_flight_phase.name == "precious_landing"
                and self.current_flight_phase.status == "reached"
            ):
                self.current_flight_phase = FlightPhase(name="landing")
                self.get_logger().info("Landing")
                self.set_mode("LAND")

            if self.current_flight_phase.name == "emergency_land":
                self.get_logger().info("EMERGENCY LANDING")
                self.set_mode("LAND")

            self.speed_pub.publish(horiz_speed)

        if not self.current_state.armed and self.current_state.mode == "LAND":
            if (
                self.current_flight_phase.name == "return_to_home"
                and not self.current_flight_phase.status
            ):
                self.get_logger().info("LANDED!")
                self.current_flight_phase = FlightPhase(name="landed")

    def calculate_intermediate_coordinates(self, point_id):
        lat_intermediate = (
            self.current_global_position.latitude
            + self.geo_points.get(point_id).latitude
        ) / 2
        lon_intermediate = (
            self.current_global_position.longitude
            + self.geo_points.get(point_id).longitude
        ) / 2
        alt_intermediate = self.current_altitude_amsl + self.intermediate_altitude
        self.intermediate_coordinates = GeoPoint(
            latitude=lat_intermediate,
            longitude=lon_intermediate,
            altitude=alt_intermediate,
        )

    def SITL_speed_correction(self):
        if self.current_state.armed and self.current_state.mode == "GUIDED":
            if (
                self.current_flight_phase.name == "descending_to_photo_altitude"
                and not self.current_flight_phase.status
            ):
                alt_delta = (
                    (self.current_altitude_home + 0.09) - self.target_photo_altitude
                )  # если коптер выше позиции, здесь положительная дельта

                if abs(alt_delta) > 0.1:
                    speed_z = -0.1
                    self.get_logger().info(
                        f"Alt Delta = {alt_delta}, speed = {speed_z}"
                    )
                    self.publish_local_speed(0, 0, speed_z)

                else:
                    self.publish_local_speed(0, 0, 0)
                    self.get_logger().info("Photo altitude reached!")
                    self.current_flight_phase.status = "reached"
                    self.get_logger().info("photo taken")
                    self.take_photo_pub.publish(
                        Int32(data=self.current_geo_point_processing)
                    )

            if (
                self.current_flight_phase.name == "ascending_from_photo_altitude"
                and not self.current_flight_phase.status
            ):
                alt_delta = (
                    (self.current_altitude_home + 0.09) - self.target_point_altitude
                )  # если коптер выше позиции, здесь положительная дельта

                if abs(alt_delta) > 0.1:
                    speed_z = 0.1
                    self.get_logger().info(
                        f"Alt Delta = {alt_delta}, speed = {speed_z}"
                    )
                    self.publish_local_speed(0, 0, speed_z)

                else:
                    self.publish_local_speed(0, 0, 0)
                    self.current_flight_phase.status = "reached"
                    self.get_logger().info("Ascending altitude reached!")

    def altitude_correction_by_rangefinder(self):
        if self.current_state.armed and self.current_state.mode == "GUIDED":
            if (
                self.current_flight_phase.name == "descending_to_photo_altitude"
                and not self.current_flight_phase.status
            ):
                alt_delta = (
                    (self.current_altitude_rangefinder + 0.09)
                    - self.target_photo_altitude
                )  # если коптер выше позиции, здесь положительная дельта

                if abs(alt_delta) > 0.1:
                    speed_z = (
                        -alt_delta / 5
                    )  # минус потому что нам надо снижаться если высота выше позиции
                    if alt_delta > 0:
                        speed_z = constrain(speed_z, -0.4, -0.15)
                    if alt_delta < 0:
                        speed_z = constrain(speed_z, 0.15, 0.4)

                    self.publish_local_speed(0, 0, speed_z)

                else:
                    self.publish_local_speed(0, 0, 0)
                    self.get_logger().info("Photo altitude reached!")
                    self.current_flight_phase.status = "reached"
                    self.get_logger().info("photo taken")
                    self.take_photo_pub.publish(
                        Int32(data=self.current_geo_point_processing)
                    )

            if (
                self.current_flight_phase.name == "taking_photo"
                and not self.current_flight_phase.status
            ):
                # Корректируем высоту пока фоткаем
                alt_delta = (
                    (self.current_altitude_rangefinder + 0.09)
                    - self.target_photo_altitude
                )  # если коптер выше позиции, здесь положительная дельта

                if abs(alt_delta) > 0.1:
                    speed_z = (
                        -alt_delta / 10
                    )  # минус потому что нам надо снижаться если высота выше позиции
                    if alt_delta > 0:
                        speed_z = constrain(speed_z, -0.15, -0.1)
                    if alt_delta < 0:
                        speed_z = constrain(speed_z, 0.1, 0.15)

                    self.publish_local_speed(0, 0, speed_z)

            if (
                self.current_flight_phase.name == "ascending_from_photo_altitude"
                and not self.current_flight_phase.status
            ):
                alt_delta = (
                    (self.current_altitude_rangefinder + 0.09)
                    - self.target_point_altitude
                )  # если коптер выше позиции, здесь положительная дельта

                if abs(alt_delta) > 0.1:
                    speed_z = alt_delta / 4
                    if alt_delta > 0:
                        speed_z = constrain(speed_z, -0.5, -0.3)
                    if alt_delta < 0:
                        speed_z = constrain(speed_z, 0.3, 0.5)

                    self.publish_local_speed(0, 0, speed_z)

                else:
                    self.publish_local_speed(0, 0, 0)
                    self.current_flight_phase.status = "reached"
                    self.get_logger().info("Ascending altitude reached!")

    def speed_position_aruco(self, msg: TwistStamped):
        dx = constrain(msg.twist.linear.x, -10, 10)
        dy = constrain(msg.twist.linear.y, -10, 10)

        vx = constrain(
            msg.twist.angular.x, -0.3, 0.3
        )  # Constrain for security purposes
        vy = constrain(msg.twist.angular.y, -0.3, 0.3)
        vz = constrain(msg.twist.angular.z, -0.3, 0.3)

        if self.current_state.armed and self.current_state.mode == "GUIDED":
            if (
                self.current_flight_phase.name == "landing_position_adjustment"
                and not self.current_flight_phase.status
            ):
                if not self.position_adjustment_timestamp:
                    self.position_adjustment_timestamp = time.monotonic()
                else:
                    now = time.monotonic()

                    if (
                        now - self.position_adjustment_timestamp
                        > self.position_adjustment_delay
                    ):
                        # время для поиска метки истекло - экстренно садимся
                        self.publish_local_speed(0, 0, 0)
                        self.get_logger().info("ARUCO not found, emergency land!")
                        self.current_flight_phase = FlightPhase(name="emergency_land")

                if abs(dx) > 1.0 and abs(dy) > 1.0:
                    self.publish_local_speed(vx, vy, 0, 0.1)
                    self.get_logger().info(
                        f"Adjusting {vx:.2f}, {vy:.2f} m/s to {dx:.2f}, {dy:.2f} m"
                    )
                else:
                    self.publish_local_speed(0, 0, 0)
                    self.current_flight_phase.status = "reached"

                    self.get_logger().info("Adjusting completed")

            if (
                self.current_flight_phase.name == "precious_landing"
                and not self.current_flight_phase.status
            ):
                if (
                    self.current_altitude_rangefinder
                    and self.current_altitude_rangefinder >= 0.6
                ):
                    self.publish_local_speed(vx, vy, vz, 0.1)
                    self.get_logger().info(
                        f"Landing {vx:.2f}, {vy:.2f}, {vz:.2f} m/s, dist={self.current_altitude_rangefinder:.2f} m to {dx:.2f}, {dy:.2f} m"
                    )

                if (
                    self.current_altitude_rangefinder
                    and self.current_altitude_rangefinder < 0.6
                ):
                    self.publish_local_speed(0, 0, 0)
                    self.get_logger().info("Landing mode will be engaged!")
                    self.current_flight_phase.status = "reached"

                if not self.current_altitude_rangefinder:
                    self.get_logger().info("RANGEFINDER not found, emergency land!")
                    self.current_flight_phase = FlightPhase(name="emergency_land")

    def publish_local_speed(self, vx: float, vy: float, vz: float, yaw=None):
        target = PositionTarget()
        target.header = Header()
        target.header.stamp = self.get_clock().now().to_msg()

        target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED

        target.type_mask = (
            PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
            | PositionTarget.IGNORE_YAW
        )

        target.velocity.x = float(vx)
        target.velocity.y = float(vy)
        target.velocity.z = float(vz)
        if yaw is not None:
            target.yaw_rate = 0.15
            target.yaw = float(yaw)
            target.type_mask = (
                PositionTarget.IGNORE_AFX
                | PositionTarget.IGNORE_AFY
                | PositionTarget.IGNORE_AFZ
            )

        self.setpoint_raw_local_pub.publish(target)

    def publish_local_position(self, x, y, z):
        self.get_logger().info(f"Moving to {x},{y},{z} meters...")

        target = PositionTarget()
        target.header = Header()
        target.header.stamp = self.get_clock().now().to_msg()

        target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED

        target.type_mask = (
            PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
            | PositionTarget.IGNORE_YAW
        )

        target.position.x = float(x)
        target.position.y = float(y)
        target.position.z = float(z)

        self.setpoint_raw_local_pub.publish(target)
        self.get_logger().info("Publishing target position...")

    def publish_yaw(self, yaw):
        self.get_logger().info(f"Rotate to {yaw} radian")

        target = PositionTarget()
        target.header = Header()
        target.header.stamp = self.get_clock().now().to_msg()

        target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED

        target.type_mask = (
            PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
        )

        target.position.x = float(0)
        target.position.y = float(0)
        target.position.z = float(0)
        target.yaw_rate = 0.3  # 0.3 rad/s = ~ 20 grad/s
        target.yaw = yaw

        self.setpoint_raw_local_pub.publish(
            target
        )  # publisher in /mavros/setpoint_raw/local
        self.get_logger().info("Publishing target position...")

    def publish_global_position(self, latitude, longitude, altitude_amsl):
        self.get_logger().info(f"Moving to {latitude}, {longitude}, {altitude_amsl}...")

        target = GlobalPositionTarget()
        target.header = Header()
        target.header.stamp = self.get_clock().now().to_msg()

        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
        target.latitude = latitude
        target.longitude = longitude
        target.altitude = altitude_amsl - self.geoid_height(latitude, longitude)

        target.type_mask = (
            GlobalPositionTarget.IGNORE_VX
            | GlobalPositionTarget.IGNORE_VY
            | GlobalPositionTarget.IGNORE_VZ
            | GlobalPositionTarget.IGNORE_AFX
            | GlobalPositionTarget.IGNORE_AFY
            | GlobalPositionTarget.IGNORE_AFZ
            | GlobalPositionTarget.IGNORE_YAW_RATE
            | GlobalPositionTarget.IGNORE_YAW
        )

        self.setpoint_raw_global_pub.publish(target)
        self.get_logger().info("Publishing target position...")

    def calculate_distance(self, point):
        """
        Calculate the great-circle distance between two points on the Earth's surface.

        Returns:
            float: Distance between the two points in meters.
        """
        # Radius of the Earth in meters
        R = 6371000

        # Convert latitude and longitude from degrees to radians
        lat1_rad = math.radians(point.latitude)
        lon1_rad = math.radians(point.longitude)
        lat2_rad = math.radians(self.current_global_position.latitude)
        lon2_rad = math.radians(self.current_global_position.longitude)

        # Difference in coordinates
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        # Haversine formula
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Calculate the distance
        distance = R * c
        self.get_logger().info(f"Distance = {distance}")
        return distance

    def check_position_reached(self):
        if (
            self.current_flight_phase.name == "move_to_intermediate"
            and not self.current_flight_phase.status
        ):
            if self.calculate_distance(self.intermediate_coordinates) <= 2.0:
                self.current_flight_phase.status = "reached"

        if (
            self.current_flight_phase.name == "move_towards_point"
            and not self.current_flight_phase.status
        ):
            if (
                self.calculate_distance(
                    self.geo_points.get(self.current_geo_point_processing)
                )
                <= 2.0
            ):
                self.current_flight_phase.status = "reached"

        if (
            self.current_flight_phase.name == "return_to_home"
            and not self.current_flight_phase.status
        ):
            if self.calculate_distance(self.home_position.geo) <= 2.0:
                self.current_flight_phase.status = "reached"

    def geoid_height(self, lat, lon):
        """Calculates AMSL to ellipsoid conversion offset.
        Uses EGM96 data with 5' grid and cubic interpolation.
        The value returned can help you convert from meters
        above mean sea level (AMSL) to meters above
        the WGS84 ellipsoid.

        If you want to go from AMSL to ellipsoid height, add the value.

        To go from ellipsoid height to AMSL, subtract this value.
        """
        return self._egm96.height(lat, lon)


def main(args=None):
    rclpy.init(args=args)
    tfl_node = TFL()

    try:
        tfl_node.get_logger().info("START node...")
        while not tfl_node.system_check.OK:
            rclpy.spin_once(tfl_node)

        tfl_node.get_logger().info("Initialized")

        # Set STABILIZE mode, arm, then set GUIDED mode
        if tfl_node.set_mode("STABILIZE"):
            time.sleep(0.2)
            if tfl_node.arm_drone():
                time.sleep(3)
                if tfl_node.set_mode("GUIDED"):
                    time.sleep(0.2)
                    # Takeoff to 10m
                    if tfl_node.takeoff():
                        # Wait for takeoff completion
                        while rclpy.ok():
                            rclpy.spin_once(tfl_node)
                            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        # drone_control_node.destroy_node()
        rclpy.shutdown()
        print("Shutdown!")


if __name__ == "__main__":
    main()
