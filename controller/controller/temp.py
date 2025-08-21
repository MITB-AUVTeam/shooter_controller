import serial
import time

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 9600 

# --- New: Motor calculation is now done in Python ---
STEPS_PER_DEGREE_X = 8.89
STEPS_PER_DEGREE_Y = 8.89

# --- Main Code ---
ser = None 
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    time.sleep(2) 
    
    startup_message = ser.readline().decode('utf-8').strip()
    if startup_message:
        print(f"Arduino says: {startup_message}")

    def send_steps(x_angle, y_angle):
        """Calculates steps from angles and sends them to the Arduino."""
        # Calculate the target steps here
        target_steps_x = int(x_angle * STEPS_PER_DEGREE_X)
        target_steps_y = int(y_angle * STEPS_PER_DEGREE_Y)

        # Format the command with step values instead of angles
        command = f"{target_steps_x} {target_steps_y}\n"
        
        ser.write(command.encode('utf-8'))
        print(f"Sent steps: X={target_steps_x}, Y={target_steps_y}")
        
        response = ser.readline().decode('utf-8').strip()
        if response:
            print(f"Response: {response}")

    # --- Main Loop ---
    while True:
        try:
            x_input = input("Enter X angle (or 'q' to quit): ")
            if x_input.lower() == 'q':
                break
            y_input = input("Enter Y angle: ")
            
            x = float(x_input)
            y = float(y_input)
            
            # This function now handles the calculation and sending
            send_steps(x, y)

        except ValueError:
            print("Invalid input. Please enter numeric values for angles.")
        except Exception as e:
            print(f"An error occurred: {e}")
            break

except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")

finally:
    if ser and ser.is_open:
        ser.close()
        print("Serial connection closed.")
