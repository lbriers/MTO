# -*- coding: utf-8 -*-
import time
import math
import board
import adafruit_lsm9ds1
import lgpio

# Pin Definitions for motors
M1_IN1 = 17
M1_IN2 = 27
M1_ENA = 12
M2_IN1 = 22
M2_IN2 = 23
M2_ENA = 13
M3_IN1 = 24
M3_IN2 = 25
M3_ENA = 18

# Motor PWM settings
PWM_FREQUENCY = 100  # Hz
MAX_SPEED = 100      # Max PWM duty cycle

# Open GPIO chip
h = lgpio.gpiochip_open(0)

# Set all pins to output
pins = [M1_IN1, M1_IN2, M1_ENA, M2_IN1, M2_IN2, M2_ENA, M3_IN1, M3_IN2, M3_ENA]
for pin in pins:
    lgpio.gpio_claim_output(h, pin, 0)

# Create sensor object
i2c = board.I2C()
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Function to set motor direction
def set_motor_direction(in1, in2, direction):
    if direction >= 0:
        lgpio.gpio_write(h, in1, 1)
        lgpio.gpio_write(h, in2, 0)
    else:
        lgpio.gpio_write(h, in1, 0)
        lgpio.gpio_write(h, in2, 1)

# Function to map heading degrees to compass direction
def get_compass_direction(degrees):
    dirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
    ix = round(degrees / 45) % 8
    return dirs[ix]

# Function to calculate and set motor speeds based on magnetometer values
def adjust_motor_speeds(mag_x, mag_y, mag_z):
    k = 300  # Sensitivity scaling factor (adjust experimentally)

    # Calculate speeds based on scaled magnetometer values
    motor_speed_1 = max(min(abs(mag_x) * k, MAX_SPEED), 0)
    motor_speed_2 = max(min(abs(mag_x) * k, MAX_SPEED), 0)
    motor_speed_3 = max(min(abs(mag_y) * k, MAX_SPEED), 0)

    # Set motor directions based on the sign
    set_motor_direction(M1_IN1, M1_IN2, mag_x)
    set_motor_direction(M2_IN1, M2_IN2, -mag_x)  # Opposite
    set_motor_direction(M3_IN1, M3_IN2, mag_y)

    # Apply PWM speeds
    lgpio.tx_pwm(h, M1_ENA, PWM_FREQUENCY, motor_speed_1)
    lgpio.tx_pwm(h, M2_ENA, PWM_FREQUENCY, motor_speed_2)
    lgpio.tx_pwm(h, M3_ENA, PWM_FREQUENCY, motor_speed_3)

# Main loop
try:
    while True:
        # Read sensor data
        accel_x, accel_y, accel_z = sensor.acceleration
        mag_x, mag_y, mag_z = sensor.magnetic
        gyro_x, gyro_y, gyro_z = sensor.gyro
        temp = sensor.temperature

        # --- Magnitude of magnetic vector ---
        magnitude = math.sqrt(mag_x**2 + mag_y**2 + mag_z**2)

        # --- Heading calculation ---
        heading_rad = math.atan2(mag_y, mag_x)
        heading_deg = math.degrees(heading_rad)
        if heading_deg < 0:
            heading_deg += 360  # Normalize to 0-360

        compass_dir = get_compass_direction(heading_deg)

        # --- Print raw magnetometer, magnitude, heading ---
        print(f"Magnetometer (gauss): X={mag_x:0.3f}, Y={mag_y:0.3f}, Z={mag_z:0.3f}")
        print(f"Magnitude: {magnitude:0.3f} gauss")
        print(f"Heading: {heading_deg:0.1f} deg ({compass_dir})")
        print("-" * 50)

        # Adjust motors based on absolute magnetometer
        adjust_motor_speeds(mag_x, mag_y, mag_z)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping motors...")
    for in1, in2, ena in [(M1_IN1, M1_IN2, M1_ENA), (M2_IN1, M2_IN2, M2_ENA), (M3_IN1, M3_IN2, M3_ENA)]:
        lgpio.gpio_write(h, in1, 0)
        lgpio.gpio_write(h, in2, 0)
        lgpio.tx_pwm(h, ena, PWM_FREQUENCY, 0)
    lgpio.gpiochip_close(h)
