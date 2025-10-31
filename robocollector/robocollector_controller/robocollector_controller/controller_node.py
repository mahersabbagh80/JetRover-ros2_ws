#!/usr/bin/env python3
"""
RoboCollector Controller Node
Main state machine coordinator for the block collection task.
Coordinates vision, navigation, and manipulation nodes.
Follows JetRover service-based architecture pattern.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from interfaces.msg import ColorInfo
from interfaces.srv import SetString
from geometry_msgs.msg import Twist
import threading
from enum import Enum


class State(Enum):
    """State machine states"""

    IDLE = 0
    SEARCHING = 1
    APPROACHING = 2
    ALIGNING = 3
    PICKING = 4
    RETURNING = 5
    PLACING = 6
    COMPLETE = 7


class RoboCollectorController(Node):
    def __init__(self, name="controller_node"):
        rclpy.init()
        super().__init__(
            name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.name = name
        self.lock = threading.RLock()

        # State machine
        self.state = State.IDLE
        self.target_color = self.get_parameter_or(
            "target_color", rclpy.Parameter("target_color", value="red")
        ).value

        # Detection tracking
        self.last_detection = None
        self.detection_count = 0
        self.required_detections = 5  # Stable detection threshold

        # Service clients
        self.vision_enter_client = None
        self.vision_exit_client = None
        self.pickup_client = None
        self.place_client = None
        self.home_client = None

        # Publishers and subscribers
        self.cmd_vel_pub = None
        self.color_info_sub = None

        # Movement parameters
        self.approach_linear_speed = 0.1  # m/s
        self.search_angular_speed = 0.3  # rad/s
        self.alignment_threshold = 50  # pixels from center

        # Service interfaces (JetRover pattern)
        self.create_service(Trigger, "~/enter", self.enter_srv_callback)
        self.create_service(Trigger, "~/exit", self.exit_srv_callback)
        self.create_service(Trigger, "~/init_finish", self.get_node_state)
        self.create_service(SetString, "~/set_target", self.set_target_callback)

        # State machine timer
        self.timer = None

        self.get_logger().info("\033[1;32m%s\033[0m" % f"{self.name} initialized")

    def enter_srv_callback(self, request, response):
        """Start RoboCollector mission"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "RoboCollector START")

        with self.lock:
            # Create service clients
            self.vision_enter_client = self.create_client(Trigger, "/vision_node/enter")
            self.vision_exit_client = self.create_client(Trigger, "/vision_node/exit")
            self.pickup_client = self.create_client(Trigger, "/pickup_node/pickup")
            self.place_client = self.create_client(Trigger, "/pickup_node/place")
            self.home_client = self.create_client(Trigger, "/pickup_node/home")

            # Create publishers and subscribers
            if self.cmd_vel_pub is None:
                self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

            if self.color_info_sub is None:
                self.color_info_sub = self.create_subscription(
                    ColorInfo, "/vision_node/color_info", self.color_info_callback, 10
                )

            # Start vision node
            self.call_service(self.vision_enter_client, Trigger.Request())

            # Initialize state machine
            self.state = State.SEARCHING
            if self.timer is None:
                self.timer = self.create_timer(0.1, self.state_machine_update)

        response.success = True
        response.message = (
            f"RoboCollector started, searching for {self.target_color} blocks"
        )
        return response

    def exit_srv_callback(self, request, response):
        """Stop RoboCollector mission"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "RoboCollector STOP")

        try:
            with self.lock:
                # Stop movement
                self.stop_robot()

                # Stop state machine
                if self.timer is not None:
                    self.destroy_timer(self.timer)
                    self.timer = None

                # Stop vision node
                if self.vision_exit_client is not None:
                    self.call_service(self.vision_exit_client, Trigger.Request())

                # Clean up publishers and subscribers
                if self.cmd_vel_pub is not None:
                    self.destroy_publisher(self.cmd_vel_pub)
                    self.cmd_vel_pub = None

                if self.color_info_sub is not None:
                    self.destroy_subscription(self.color_info_sub)
                    self.color_info_sub = None

                # Reset state
                self.state = State.IDLE
                self.last_detection = None
                self.detection_count = 0

        except Exception as e:
            self.get_logger().error(f"Exit error: {str(e)}")

        response.success = True
        response.message = "RoboCollector stopped"
        return response

    def get_node_state(self, request, response):
        """Return node initialization state"""
        response.success = True
        return response

    def set_target_callback(self, request, response):
        """Change target color"""
        self.target_color = request.data
        self.get_logger().info(f"Target color set to: {self.target_color}")
        response.success = True
        response.message = f"Target set to {self.target_color}"
        return response

    def color_info_callback(self, msg):
        """Receive color detection information"""
        with self.lock:
            if msg.color == self.target_color:
                self.last_detection = msg
                self.detection_count += 1
            else:
                self.detection_count = 0

    def state_machine_update(self):
        """Main state machine loop"""
        with self.lock:
            if self.state == State.SEARCHING:
                self.state_searching()
            elif self.state == State.APPROACHING:
                self.state_approaching()
            elif self.state == State.ALIGNING:
                self.state_aligning()
            elif self.state == State.PICKING:
                self.state_picking()
            elif self.state == State.RETURNING:
                self.state_returning()
            elif self.state == State.PLACING:
                self.state_placing()
            elif self.state == State.COMPLETE:
                self.state_complete()

    def state_searching(self):
        """Rotate to search for blocks"""
        if self.detection_count >= self.required_detections:
            self.get_logger().info(
                "\033[1;32m%s\033[0m" % "Block detected! Approaching..."
            )
            self.state = State.APPROACHING
            self.stop_robot()
        else:
            # Rotate slowly to search
            self.rotate_robot(self.search_angular_speed)

    def state_approaching(self):
        """Move toward the detected block"""
        if self.last_detection is None:
            self.get_logger().warn("Lost detection, returning to search")
            self.state = State.SEARCHING
            return

        # Calculate image center
        img_center_x = self.last_detection.width / 2
        detection_x = self.last_detection.x

        # Check if aligned
        error_x = detection_x - img_center_x

        if abs(error_x) > self.alignment_threshold:
            self.state = State.ALIGNING
        else:
            # Move forward (placeholder - would use depth for distance)
            self.get_logger().info("Aligned! Moving to pickup position")
            self.stop_robot()
            self.state = State.PICKING

    def state_aligning(self):
        """Align with the block"""
        if self.last_detection is None:
            self.state = State.SEARCHING
            return

        img_center_x = self.last_detection.width / 2
        detection_x = self.last_detection.x
        error_x = detection_x - img_center_x

        if abs(error_x) <= self.alignment_threshold:
            self.get_logger().info("Alignment complete")
            self.stop_robot()
            self.state = State.APPROACHING
        else:
            # Proportional control for alignment
            angular_vel = -error_x * 0.001
            self.rotate_robot(angular_vel)

    def state_picking(self):
        """Execute pickup sequence"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "Picking up block")
        self.stop_robot()

        # Call pickup service
        result = self.call_service(self.pickup_client, Trigger.Request())

        if result and result.success:
            self.get_logger().info("Pickup successful! Returning to base")
            self.state = State.RETURNING
        else:
            self.get_logger().error("Pickup failed, returning to search")
            self.state = State.SEARCHING

    def state_returning(self):
        """Navigate back to drop-off location"""
        self.get_logger().info("Returning to base...")
        # TODO: Integrate with navigation stack
        # For now, just proceed to placing
        self.state = State.PLACING

    def state_placing(self):
        """Place the block in the collection box"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "Placing block")

        result = self.call_service(self.place_client, Trigger.Request())

        if result and result.success:
            self.get_logger().info("Place successful!")
            self.state = State.COMPLETE
        else:
            self.get_logger().error("Place failed")
            self.state = State.COMPLETE

    def state_complete(self):
        """Mission complete"""
        self.get_logger().info("\033[1;32m%s\033[0m" % "Mission complete!")
        self.stop_robot()
        # Reset for next run
        self.state = State.SEARCHING
        self.detection_count = 0

    def stop_robot(self):
        """Stop all robot movement"""
        if self.cmd_vel_pub is not None:
            msg = Twist()
            self.cmd_vel_pub.publish(msg)

    def rotate_robot(self, angular_vel):
        """Rotate robot at specified angular velocity"""
        if self.cmd_vel_pub is not None:
            msg = Twist()
            msg.angular.z = angular_vel
            self.cmd_vel_pub.publish(msg)

    def move_robot(self, linear_vel, angular_vel=0.0):
        """Move robot with specified velocities"""
        if self.cmd_vel_pub is not None:
            msg = Twist()
            msg.linear.x = linear_vel
            msg.angular.z = angular_vel
            self.cmd_vel_pub.publish(msg)

    def call_service(self, client, request):
        """Helper to call a service synchronously"""
        if client is None:
            self.get_logger().error("Service client is None")
            return None

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service {client.srv_name} not available")
            return None

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done():
            return future.result()
        else:
            self.get_logger().error("Service call timed out")
            return None


def main(args=None):
    node = RoboCollectorController("controller_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
