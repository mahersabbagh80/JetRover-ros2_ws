#!/usr/bin/env python3
"""
RoboCollector Vision Node
Detects colored blocks using HSV color space and publishes their positions.
Follows JetRover service-based architecture pattern.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from interfaces.msg import ColorInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading


class VisionNode(Node):
    def __init__(self, name="vision_node"):
        rclpy.init()
        super().__init__(
            name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.name = name
        self.lock = threading.RLock()
        self.bridge = CvBridge()

        # Vision parameters
        self.debug = self.get_parameter_or(
            "debug", rclpy.Parameter("debug", value=False)
        ).value
        self.target_color = self.get_parameter_or(
            "target_color", rclpy.Parameter("target_color", value="red")
        ).value

        # HSV color ranges for different blocks
        self.color_ranges = {
            "red": [
                (np.array([0, 120, 70]), np.array([10, 255, 255])),
                (np.array([170, 120, 70]), np.array([180, 255, 255])),
            ],
            "blue": [(np.array([100, 120, 70]), np.array([130, 255, 255]))],
            "green": [(np.array([40, 120, 70]), np.array([80, 255, 255]))],
            "yellow": [(np.array([20, 120, 70]), np.array([40, 255, 255]))],
        }

        # Publishers and subscribers (initialized when entering)
        self.image_sub = None
        self.result_pub = None
        self.color_info_pub = None

        # Detection state
        self.last_detection = None
        self.min_area = 500  # Minimum contour area for valid detection

        # Service interfaces (JetRover pattern)
        self.create_service(Trigger, "~/enter", self.enter_srv_callback)
        self.create_service(Trigger, "~/exit", self.exit_srv_callback)
        self.create_service(Trigger, "~/init_finish", self.get_node_state)

        self.get_logger().info("\033[1;32m%s\033[0m" % f"{self.name} initialized")

    def enter_srv_callback(self, request, response):
        """Start vision processing when app enters"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "vision enter")
        with self.lock:
            if self.image_sub is None:
                self.image_sub = self.create_subscription(
                    Image, "/depth_cam/rgb/image_raw", self.image_callback, 10
                )
            if self.result_pub is None:
                self.result_pub = self.create_publisher(Image, "~/result_image", 10)
            if self.color_info_pub is None:
                self.color_info_pub = self.create_publisher(
                    ColorInfo, "~/color_info", 10
                )

        response.success = True
        response.message = "Vision node started"
        return response

    def exit_srv_callback(self, request, response):
        """Stop vision processing when app exits"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "vision exit")
        try:
            with self.lock:
                if self.image_sub is not None:
                    self.destroy_subscription(self.image_sub)
                    self.image_sub = None
                if self.result_pub is not None:
                    self.destroy_publisher(self.result_pub)
                    self.result_pub = None
                if self.color_info_pub is not None:
                    self.destroy_publisher(self.color_info_pub)
                    self.color_info_pub = None
                self.last_detection = None
        except Exception as e:
            self.get_logger().error(f"Exit error: {str(e)}")

        response.success = True
        response.message = "Vision node stopped"
        return response

    def get_node_state(self, request, response):
        """Return node initialization state"""
        response.success = True
        return response

    def image_callback(self, ros_image):
        """Process camera images for color detection"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
            result_image = np.copy(cv_image)

            with self.lock:
                result_image = self.detect_color(cv_image, result_image)

            # Debug visualization
            if self.debug:
                cv2.imshow(
                    "Color Detection", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
                )
                cv2.waitKey(1)

            # Publish result image
            if self.result_pub is not None:
                self.result_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))

        except Exception as e:
            self.get_logger().error(f"Image callback error: {str(e)}")

    def detect_color(self, cv_image, result_image):
        """Detect colored blocks in the image"""
        # Convert to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

        # Get color range for target color
        if self.target_color not in self.color_ranges:
            self.get_logger().warn(f"Unknown color: {self.target_color}")
            return result_image

        # Create mask for target color
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self.color_ranges[self.target_color]:
            mask |= cv2.inRange(hsv, lower, upper)

        # Morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > self.min_area:
                # Calculate center and bounding box
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Draw bounding box and center
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(result_image, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(
                        result_image,
                        f"{self.target_color}: {area:.0f}px",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

                    # Publish color info
                    if self.color_info_pub is not None:
                        color_msg = ColorInfo()
                        color_msg.color = self.target_color
                        color_msg.x = cx
                        color_msg.y = cy
                        color_msg.width = result_image.shape[1]
                        color_msg.height = result_image.shape[0]
                        self.color_info_pub.publish(color_msg)

                    self.last_detection = (cx, cy, area)
                    self.get_logger().debug(
                        f"Detected {self.target_color} at ({cx}, {cy}), area: {area}"
                    )

        return result_image


def main(args=None):
    node = VisionNode("vision_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
