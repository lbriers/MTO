# -*- coding: utf-8 -*-
import time
import math
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

# Delta platform geometry (in meters)
r_w = 0.03   # wheel radius = 30 mm
r_b = 0.15   # base link offset = 150 mm

# Platform link angles in radians
theta_1 = 0
theta_2 = -120 * math.pi / 180
theta_3 = -240 * math.pi / 180  # equivalent to +120° in unit circle

# Open GPIO chip
h = lgpio.gpiochip_open(0)

# Set all motor control pins to output
pins = [M1_IN1, M1_IN2, M1_ENA, M2_IN1, M2_IN2, M2_ENA, M3_IN1, M3_IN2, M3_ENA]
for pin in pins:
    lgpio.gpio_claim_output(h, pin, 0)

def compute_motor_speeds(vel_ref, theta2_ref, omega_z_ref):
    """
    Calculate the motor angular speeds from platform motion.
    vel_ref: linear speed of the platform [m/s]
    theta2_ref: orientation of platform [rad]
    omega_z_ref: angular speed around Z axis [rad/s]
    Returns list of [ω1, ω2, ω3] in rad/s
    """
    motor_speeds = []
    angles = [theta_1, theta_2, theta_3]

    for angle in angles:
        projection = vel_ref * math.cos(angle - theta2_ref)
        omega_m = (1 / r_w) * (projection + omega_z_ref * r_b)
        motor_speeds.append(omega_m)
    
    return motor_speeds

def set_motor_pwm(pin_in1, pin_in2, pin_ena, speed):
    """
    Control motor direction and speed.
    speed: PWM magnitude, range -100 to 100
    """
    direction = 1 if speed >= 0 else 0
    duty_cycle = min(abs(speed), MAX_SPEED)

    lgpio.gpio_write(h, pin_in1, direction)
    lgpio.gpio_write(h, pin_in2, 1 - direction)
    lgpio.tx_pwm(h, pin_ena, PWM_FREQUENCY, duty_cycle)

def drive_motors(motor_speeds):
    """
    Drive all three motors using calculated angular speeds.
    """
    motor_pins = [
        (M1_IN1, M1_IN2, M1_ENA),
        (M2_IN1, M2_IN2, M2_ENA),
        (M3_IN1, M3_IN2, M3_ENA)
    ]
    for i in range(3):
        # Convert ω [rad/s] to approximate PWM duty cycle (-100 to +100)
        pwm_speed = motor_speeds[i] * 50  # Scaling factor (adjust if needed)
        set_motor_pwm(*motor_pins[i], pwm_speed)

def stop_motors():
    """
    Stop all motors by setting PWM to 0.
    """
    for pin in [M1_ENA, M2_ENA, M3_ENA]:
        lgpio.tx_pwm(h, pin, PWM_FREQUENCY, 0)

# Main test loop
try:
    while True:
        # Example platform motion: forward + rotate
        vel_ref = 0.05             # m/s linear velocity
        theta2_ref = 0             # platform pointing forward
        omega_z_ref = 0.2          # rad/s angular velocity

        motor_speeds = compute_motor_speeds(vel_ref, theta2_ref, omega_z_ref)
        drive_motors(motor_speeds)

        time.sleep(0.1)

except KeyboardInterrupt:
    stop_motors()
    print("Motors stopped.")
    lgpio.gpiochip_close(h)
