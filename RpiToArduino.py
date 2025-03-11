import serial
import time

# Open serial connection to Arduino
ser = serial.Serial('/dev/serial0', 115200, timeout=1)  # Use /dev/ttyUSB0 if using USB-Serial adapter
time.sleep(2)  # Wait for connection to establish

def set_motor_speed(duty_cycle):
    """Send the duty cycle command to Arduino over UART."""
    command = f"{duty_cycle}\n"  # Ensure newline for proper parsing on Arduino
    ser.write(command.encode('utf-8'))
    print(f"Sent: {command.strip()}")

try:
    while True:
        speed = input("Enter motor speed (0-100%): ")
        if speed.isdigit():
            duty_cycle = int(speed)
            if 0 <= duty_cycle <= 100:
                set_motor_speed(duty_cycle)
            else:
                print("Enter a value between 0 and 100.")
        else:
            print("Invalid input, enter a number.")

except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
