#!/usr/bin/env python3
"""
RoboCollector Simulation Test Node
Simulates camera feed with colored blocks for testing vision and controller
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class SimulationTestNode(Node):
    def __init__(self):
        super().__init__("simulation_test_node")

        self.bridge = CvBridge()

        # Create publisher for simulated camera
        self.image_pub = self.create_publisher(Image, "/depth_cam/rgb/image_raw", 10)

        # Timer to publish test images
        self.timer = self.create_timer(0.1, self.publish_test_image)

        # Test scenario parameters
        self.frame_count = 0
        self.block_x = 320  # Center of 640x480 image
        self.block_y = 240
        self.block_moving = False

        self.get_logger().info("Simulation test node started")
        self.get_logger().info("Publishing test images to /depth_cam/rgb/image_raw")

    def publish_test_image(self):
        """Generate and publish synthetic test images with colored blocks"""

        # Create blank image (640x480)
        image = np.ones((480, 640, 3), dtype=np.uint8) * 200  # Light gray background

        # Draw floor
        cv2.rectangle(image, (0, 350), (640, 480), (180, 180, 180), -1)

        # Animate block movement (simulate robot approaching)
        if self.frame_count < 200:
            # Block starts small and far
            scale = 50 + self.frame_count
            self.block_x = 320 + int(30 * np.sin(self.frame_count * 0.05))
        else:
            # Block gets larger (closer)
            scale = 100
            self.block_x = 320

        # Draw colored block (red by default)
        color = (255, 0, 0)  # Red in RGB
        block_size = int(scale)
        top_left = (self.block_x - block_size // 2, self.block_y - block_size // 2)
        bottom_right = (self.block_x + block_size // 2, self.block_y + block_size // 2)

        cv2.rectangle(image, top_left, bottom_right, color, -1)
        cv2.rectangle(image, top_left, bottom_right, (0, 0, 0), 2)  # Black border

        # Add some noise for realism
        noise = np.random.randint(-10, 10, image.shape, dtype=np.int16)
        image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Publish image
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_rgb_optical_frame"
        self.image_pub.publish(ros_image)

        self.frame_count += 1

        # Reset after full cycle
        if self.frame_count > 400:
            self.frame_count = 0
            self.get_logger().info("Simulation cycle complete, restarting...")


def main(args=None):
    rclpy.init(args=args)
    node = SimulationTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
