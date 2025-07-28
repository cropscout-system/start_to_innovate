#!/usr/bin/env python3

from math import atan2, cos, degrees, hypot, pi, sin, sqrt

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector_node")

        self.declare_parameter("marker_length", 23.0)  # ArUco marker size in cm
        self.declare_parameter(
            "camera_gamma", 1.835
        )  # Camera parameter for distance estimation
        self.declare_parameter("ids_to_detect", [5, 6, 7])  # ArUco marker IDs to detect
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("visualization_enabled", False)

        self.marker_length = self.get_parameter("marker_length").value
        self.gamma = self.get_parameter("camera_gamma").value
        self.ids_to_detect = self.get_parameter("ids_to_detect").value
        self.visualization_enabled = self.get_parameter("visualization_enabled").value

        self.cv_bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.pose_pub = self.create_publisher(PoseStamped, "/drone/aruco/pose", 10)
        self.markers_data_pub = self.create_publisher(
            Float32MultiArray, "/drone/aruco/markers", 10
        )
        self.speed_position_pub = self.create_publisher(
            TwistStamped, "/drone/aruco/speed_position", 10
        )

        if self.visualization_enabled:
            self.visualization_pub = self.create_publisher(
                Image, "/drone/aruco/visualization", 10
            )

        self.camera_sub = self.create_subscription(
            Image, self.get_parameter("camera_topic").value, self.image_callback, 10
        )

        cv2.setNumThreads(2)

        self.get_logger().info("ArUco detector node initialized")

    def poly_area(self, x, y):
        return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

    def image_callback(self, msg):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            height, width = frame.shape[:2]
            center_x, center_y = width // 2, height // 2

            corners, ids, _ = self.detector.detectMarkers(frame_gray)

            if self.visualization_enabled:
                vis_frame = frame.copy()
                cv2.circle(vis_frame, (center_x, center_y), 4, (0, 0, 0), -1)

            if len(corners) > 0 and ids is not None:
                ids = ids.flatten()

                for marker_corner, marker_id in zip(corners, ids, strict=False):
                    if marker_id in self.ids_to_detect:
                        corners4 = marker_corner.reshape((4, 2))
                        top_left, _, bottom_right, _ = corners4

                        marker_center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                        marker_center_y = int((top_left[1] + bottom_right[1]) / 2.0)

                        x = corners4[:, 1:].ravel()
                        y = corners4[:, :1].ravel()
                        S = self.poly_area(x, y)

                        # Distance from image center to marker center
                        px_dist_from_c = hypot(
                            marker_center_x - center_x, marker_center_y - center_y
                        )

                        # Calculate height and horizontal distance
                        alpha = sqrt(S / (width * height)) * self.gamma
                        height_cm = self.marker_length / alpha
                        distance_cm = px_dist_from_c / sqrt(S) * self.marker_length

                        # Calculate angle to marker
                        theta = atan2(
                            -1 * (marker_center_x - center_x),
                            -1 * (marker_center_y - center_y),
                        )
                        theta += int(theta < 0) * 2 * pi  # Normalize to [0, 2π]

                        # Calculate marker position relative to camera
                        d_x = -1 * sin(theta) * distance_cm
                        d_y = cos(theta) * distance_cm

                        # Calculate marker orientation
                        phi = atan2(y[1] - y[0], x[1] - x[0])
                        phi += int(phi < 0) * 2 * pi  # Normalize to [0, 2π]

                        marker_data = Float32MultiArray()
                        marker_data.data = [
                            float(marker_id),
                            height_cm,
                            distance_cm,
                            degrees(theta),
                            d_x,
                            d_y,
                            degrees(phi),
                        ]
                        self.markers_data_pub.publish(marker_data)

                        D_vector = Vector3()
                        D_vector.x = d_x / 100.0  # m
                        D_vector.y = d_y / 100.0  # m
                        D_vector.z = height_cm / 100.0  # m

                        speed_vector = self.calculate_speed(D_vector)

                        data_msg = TwistStamped()
                        data_msg.header.stamp = self.get_clock().now().to_msg()
                        data_msg.header.frame_id = "camera_frame"
                        data_msg.twist.linear = D_vector
                        data_msg.twist.angular = speed_vector

                        self.speed_position_pub.publish(data_msg)

                        self.get_logger().info(
                            f"Detected marker ID {marker_id}: "
                            f"height={height_cm:.1f}cm, "
                            f"distance={distance_cm:.1f}cm, "
                            f"dx={d_x:.1f}cm, dy={d_y:.1f}cm"
                        )

                        if self.visualization_enabled:
                            cv2.aruco.drawDetectedMarkers(vis_frame, corners)
                            cv2.circle(
                                vis_frame,
                                (marker_center_x, marker_center_y),
                                4,
                                (0, 0, 255),
                                -1,
                            )

                            cv2.line(
                                vis_frame,
                                (marker_center_x, marker_center_y),
                                (center_x, center_y),
                                (0, 0, 255),
                                1,
                            )

                            cv2.putText(
                                vis_frame,
                                f"Height: {height_cm:.1f} cm",
                                (marker_center_x - 15, marker_center_y - 15),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (255, 0, 255),
                                2,
                            )
                            cv2.putText(
                                vis_frame,
                                f"Dist: {distance_cm:.1f} cm",
                                (
                                    (center_x + marker_center_x) // 2,
                                    (center_y + marker_center_y) // 2,
                                ),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 0, 255),
                                2,
                            )
                            cv2.putText(
                                vis_frame,
                                f"{degrees(theta):.1f} degrees",
                                (center_x, center_y - 15),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 0, 255),
                                2,
                            )

            if self.visualization_enabled:
                vis_msg = self.cv_bridge.cv2_to_imgmsg(vis_frame, encoding="bgr8")
                self.visualization_pub.publish(vis_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e!s}")

    def calculate_speed(self, D: Vector3):
        x_position = D.x
        y_position = D.y

        z_speed = -0.1

        x_speed = abs(x_position) / 5.0
        y_speed = abs(y_position) / 5.0

        x_speed = max(0.1, min(x_speed, 0.25))
        y_speed = max(0.1, min(y_speed, 0.25))

        if x_position < 0:
            x_speed = -x_speed
        if y_position < 0:
            y_speed = -y_speed

        speed_msg = Vector3()
        speed_msg.x = float(x_speed)
        speed_msg.y = float(y_speed)
        speed_msg.z = float(z_speed)

        return speed_msg


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
