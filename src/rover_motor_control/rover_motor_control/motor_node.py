#!/usr/bin/env python3

import time
import serial
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


class RoverMotorController(Node):

    def __init__(self):
        super().__init__('rover_motor_controller')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation', 0.40)
        self.declare_parameter('max_linear_speed', 0.50)
        self.declare_parameter('max_pwm', 600)
        self.declare_parameter('turn_pwm_gain', 800.0)

        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value
        )
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_pwm = int(
            self.get_parameter('max_pwm').value
        )
        self.turn_pwm_gain = float(
            self.get_parameter('turn_pwm_gain').value
        )

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baud_rate').value)

        self.last_command_time = time.monotonic()

        # Connect to ESP8266
        try:
            self.serial = serial.Serial(
                port,
                baud,
                timeout=0.1,
                write_timeout=0.1
            )

            # Allow the ESP8266 to reset after opening serial
            time.sleep(2)

            # Ensure motors start stopped
            self.serial.write(b'0,0\n')

            self.get_logger().info(
                f'Connected to ESP8266 on {port}'
            )

        except serial.SerialException as error:
            self.get_logger().error(
                f'Could not open serial port {port}: {error}'
            )
            raise

        # Subscribe to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Stop motors when commands become stale
        self.watchdog = self.create_timer(
            0.05,
            self.watchdog_callback
        )

        self.get_logger().info(
            'Motor controller ready; listening to /cmd_vel'
        )

    def send_motor_command(self, left_pwm, right_pwm):
        """Send signed PWM values to the ESP8266."""

        left_pwm = max(-1023, min(1023, int(left_pwm)))
        right_pwm = max(-1023, min(1023, int(right_pwm)))

        # Reverse the right-side motor direction
        right_pwm = -right_pwm

        command = f'{left_pwm},{right_pwm}\n'

        try:
            self.serial.write(command.encode('ascii'))

        except serial.SerialException as error:
            self.get_logger().error(
                f'Failed to send motor command: {error}'
            )

    def cmd_vel_callback(self, msg):
        """Convert ROS 2 velocity commands to motor PWM."""

        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        # In-place turning:
        # Left and right sides receive opposite PWM commands.
        if abs(linear) < 0.001 and abs(angular) > 0.001:

            turn_pwm = angular * self.turn_pwm_gain

            # Limit the turning PWM to the configured safe range
            turn_pwm = max(
                -self.max_pwm,
                min(self.max_pwm, turn_pwm)
            )

            left_pwm = -turn_pwm
            right_pwm = turn_pwm

        else:
            # Differential-drive movement
            left_velocity = (
                linear
                + angular * self.wheel_separation / 2.0
            )

            right_velocity = (
                linear
                - angular * self.wheel_separation / 2.0
            )

            # Keep wheel velocities within the configured limit
            peak_velocity = max(
                abs(left_velocity),
                abs(right_velocity)
            )

            if peak_velocity > self.max_linear_speed:
                scale = self.max_linear_speed / peak_velocity

                left_velocity *= scale
                right_velocity *= scale

            # Convert velocities to PWM
            left_pwm = (
                left_velocity / self.max_linear_speed
            ) * self.max_pwm

            right_pwm = (
                right_velocity / self.max_linear_speed
            ) * self.max_pwm

        # Send commands to the ESP8266
        self.send_motor_command(left_pwm, right_pwm)

        # Record when the last valid velocity command arrived
        self.last_command_time = time.monotonic()

    def watchdog_callback(self):
        """Stop motors if velocity commands stop arriving."""

        if time.monotonic() - self.last_command_time > 0.30:
            self.send_motor_command(0, 0)

    def destroy_node(self):
        """Stop motors and close serial communication."""

        try:
            self.serial.write(b'0,0\n')
            self.serial.close()

        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = RoverMotorController()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        print(f'Motor node failed: {error}')

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
