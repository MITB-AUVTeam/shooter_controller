import rclpy
from rclpy.node import Node
import serial
import time

from custom_interfaces.msg import Pose   # CHANGE


class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.subscription = self.create_subscription(
            Pose,                # CHANGE
            'orient_ctrl',
            self.listener_callback,
            10)
        self.subscription

        # --- Serial Configuration ---
        self.SERIAL_PORT = '/dev/ttyACM0' 
        self.BAUD_RATE = 9600 

        # --- Steps per degree ---
        self.STEPS_PER_DEGREE_X = 8.89
        self.STEPS_PER_DEGREE_Y = 8.89

        # --- Open Serial ---
        self.ser = None 
        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=1)
            self.get_logger().info(f"Connecting to {self.SERIAL_PORT} at {self.BAUD_RATE} baud...")
            time.sleep(2) 
            
            startup_message = self.ser.readline().decode('utf-8').strip()
            if startup_message:
                self.get_logger().info(f"Arduino says: {startup_message}")

        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port {self.SERIAL_PORT}: {e}")

    def send_steps(self, x_angle, y_angle):
        """Convert relative angles to relative steps and send to Arduino."""
        delta_steps_x = int(x_angle * self.STEPS_PER_DEGREE_X)
        delta_steps_y = int(y_angle * self.STEPS_PER_DEGREE_Y)

        command = f"{delta_steps_x} {delta_steps_y}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent relative steps: X={delta_steps_x}, Y={delta_steps_y}")

        response = self.ser.readline().decode('utf-8').strip()
        if response:
            self.get_logger().info(f"Response: {response}")

    def listener_callback(self, msg):
        try:            
            yaw_angle = msg.yaw_theta
            pitch_angle = msg.pitch_theta
            
            # Send relative steps (like updated standalone code)
            self.send_steps(yaw_angle, pitch_angle)

        except ValueError:
            self.get_logger().warn("Invalid input. Please enter numeric values for angles.")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def destroy_node(self):
        # Close serial when shutting down
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
