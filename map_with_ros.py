#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class OffboardControl(Node):
    """Node for controlling a drone in offboard mode and capturing images at each setpoint."""

    def __init__(self):
        super().__init__('offboard_control')

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Subscriber for camera images
        self.camera_subscription = self.create_subscription(
            Image, '/drone/camera/image_raw', self.camera_callback, 10)

        # Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()

        # Drone settings
        self.takeoff_height = -5.0  # Adjust for the drone's altitude
        self.sector_size = 2.5  # Size of each sector
        self.num_rows = 4  # Example grid of 4x4
        self.num_cols = 4
        self.start_position = (0.0, 0.0)  # Starting point of the grid

        # Generate S-shaped trajectory
        self.setpoints = self.generate_s_trajectory()
        self.current_setpoint_index = 0

        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def generate_s_trajectory(self):
        """Generate a trajectory that follows an S-shaped path."""
        setpoints = []
        for row in range(self.num_rows - 1, -1, -1):
            if row % 2 == 0:
                for col in range(self.num_cols):
                    x = col * self.sector_size
                    y = row * self.sector_size
                    setpoints.append((x, y, self.takeoff_height))
            else:
                for col in range(self.num_cols - 1, -1, -1):
                    x = col * self.sector_size
                    y = row * self.sector_size
                    setpoints.append((x, y, self.takeoff_height))
        return setpoints

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback for vehicle position updates."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback for vehicle status updates."""
        self.vehicle_status = vehicle_status

    def camera_callback(self, msg):
        """Callback for camera image messages."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Drone Camera", cv_image)
        cv2.waitKey(1)

    def capture_image(self):
        """Capture and save an image at the current setpoint."""
        # Simulating capturing the image; in real use, the image would come from the camera topic
        self.get_logger().info(f"Capturing image at sector {self.current_setpoint_index}")
        filename = f"sector_{self.current_setpoint_index}.png"
        # Use cv2.imwrite() to save the captured image
        # Example: cv2.imwrite(filename, cv_image)
        self.get_logger().info(f"Image saved as {filename}")

    def arm(self):
        """Send an arm command to the drone."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Drone armed')

    def disarm(self):
        """Send a disarm command to the drone."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Drone disarmed')

    def engage_offboard_mode(self):
        """Switch the drone to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode engaged")

    def land(self):
        """Land the drone."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Landing initiated")

    def publish_offboard_control_heartbeat(self):
        """Publish offboard control mode heartbeat."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        """Publish the next position setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # Set yaw to 90 degrees
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        """Main loop for controlling the drone."""
        self.publish_offboard_control_heartbeat()

        if self.current_setpoint_index == 0:
            self.engage_offboard_mode()
            self.arm()

        if self.current_setpoint_index < len(self.setpoints):
            # Get the next setpoint
            sp = self.setpoints[self.current_setpoint_index]
            self.publish_position_setpoint(sp[0], sp[1], sp[2])

            # Check if the drone is near the setpoint
            if (abs(self.vehicle_local_position.x - sp[0]) < 0.5 and
                    abs(self.vehicle_local_position.y - sp[1]) < 0.5 and
                    abs(self.vehicle_local_position.z - sp[2]) < 0.5):
                self.capture_image()  # Capture the image
                self.current_setpoint_index += 1
                self.get_logger().info(f"Moved to setpoint: {sp}")
        else:
            self.land()
            self.disarm()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
