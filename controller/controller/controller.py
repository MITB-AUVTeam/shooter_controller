import rclpy
from rclpy.node import Node
import serial
import time

from custom_interfaces.msg import Pose                     # CHANGE


class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.subscription = self.create_subscription(
            Pose,                                               # CHANGE
            'orient_ctrl',
            self.listener_callback,
            10)
        self.subscription


        self.SERIAL_PORT = '/dev/ttyACM0' 
        self.BAUD_RATE = 9600 

        # --- New: Motor calculation is now done in Python ---
        self.STEPS_PER_DEGREE_X = 8.89
        self.STEPS_PER_DEGREE_Y = 8.89
        self.ser = None 
        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=1)
            self.get_logger().info(f"Connecting to {self.SERIAL_PORT} at {self.BAUD_RATE} baud...")
            time.sleep(2) 
            
            startup_message = self.ser.readline().decode('utf-8').strip()
            if startup_message:
                self.get_logger().info(f"Arduino says: {startup_message}")

        except serial.SerialException as e:
            self.get_logger().info(f"Error opening serial port {self.SERIAL_PORT}: {e}")

    def send_steps(self,x_angle, y_angle):
        """Calculates steps from angles and sends them to the Arduino."""
        # Calculate the target steps here
        target_steps_x = int(x_angle * self.STEPS_PER_DEGREE_X)
        target_steps_y = int(y_angle * self.STEPS_PER_DEGREE_Y)

        # Format the command with step values instead of angles
        command = f"{target_steps_x} {target_steps_y}\n"
        
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent steps: yaw_angle={target_steps_x}, pitch_angle={target_steps_y}")
        
        response = self.ser.readline().decode('utf-8').strip()
        if response:
            self.get_logger().info(f"Response: {response}")

    def listener_callback(self, msg):
        try:            
            yaw_angle = msg.yaw_theta
            pitch_angle = msg.pitch_theta
            
            # This function now handles the calculation and sending
            self.send_steps(yaw_angle, pitch_angle)

        except ValueError:
            self.get_logger().info("Invalid input. Please enter numeric values for angles.")
        except Exception as e:
            self.get_logger().info(f"An error occurred: {e}")

        


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()