# -*- coding: utf-8 -*-
import time
import math
import lgpio

import board
import adafruit_lsm9ds1
import numpy as np
import sys
# Import Madgwick library directly
from madgwick_py.quaternion import Quaternion
from madgwick_py.madgwickahrs import MadgwickAHRS

# Create sensor object
i2c = board.I2C()
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Initialize Madgwick filter - keeping your original parameters
sample_period = 0.01  
madgwick = MadgwickAHRS(sampleperiod=sample_period, beta=0.3)

# Magnetometer calibration parameters
mag_offset = (0, 0, 0)  # calibrated offsets (bias)
mag_scale = (1, 1, 1)   # Calibrated scaling factors
# Motor pin definitions
M1_IN1 = 22
M1_IN2 = 23
M1_ENA = 13
M2_IN1 = 17
M2_IN2 = 27
M2_ENA = 12
M3_IN1 = 24
M3_IN2 = 25
M3_ENA = 18

PWM_FREQUENCY = 100  # Hz
MAX_SPEED = 100      # Max PWM duty cycle

# Delta platform geometry (in meters)
r_w = 0.03    # wheel radius
r_b = 0.132   # base radius (distance from center to wheel)

# Platform wheel angles (in radians)
theta_1 = 0
theta_2 = 120 * math.pi / 180
theta_3 = 240 * math.pi / 180

# RPM conversion constants
RAD_S_TO_RPM = 60 / (2 * math.pi)  # ˜ 9.5493
RPM_TO_RAD_S = (2 * math.pi) / 60  # ˜ 0.10472
MAX_RPM = 188                      # RPM at 100% duty cycle

# GPIO setup
h = lgpio.gpiochip_open(0)
pins = [M1_IN1, M1_IN2, M1_ENA, M2_IN1, M2_IN2, M2_ENA, M3_IN1, M3_IN2, M3_ENA]
for pin in pins:
    lgpio.gpio_claim_output(h, pin, 0)

def compute_motor_rpms(vel_ref, theta_ref, omega_z_ref):
    """
    Compute motor speeds in RPM:
    omega_i_rad = (1/r_w) * (v*cos(theta_i - theta_ref) + omega_z * r_b)
    omega_i_rpm = omega_i_rad * 60 / (2*pi)
    """
    motor_rpms = []
    angles = [theta_1, theta_2, theta_3]
    
    for angle in angles:
        linear_component = vel_ref * math.cos(angle - theta_ref)
        angular_component = omega_z_ref * r_b
        wheel_omega_rad = (1 / r_w) * (linear_component + angular_component)
        wheel_omega_rpm = wheel_omega_rad * RAD_S_TO_RPM
        motor_rpms.append(wheel_omega_rpm)
    
    return motor_rpms

def set_motor_pwm(pin_in1, pin_in2, pin_ena, motor_rpm):
    """Set motor direction and PWM duty based on RPM value"""
    duty_cycle = min(abs(motor_rpm) / MAX_RPM * 100, MAX_SPEED)
    direction = 1 if motor_rpm >= 0 else 0
    lgpio.gpio_write(h, pin_in1, direction)
    lgpio.gpio_write(h, pin_in2, 1 - direction)
    lgpio.tx_pwm(h, pin_ena, PWM_FREQUENCY, duty_cycle)

def drive_motors(motor_rpms):
    motor_pins = [
        (M1_IN1, M1_IN2, M1_ENA),
        (M2_IN1, M2_IN2, M2_ENA),
        (M3_IN1, M3_IN2, M3_ENA)
    ]
    print("\nMotor Commands:")
    for i in range(3):
        duty = min(abs(motor_rpms[i]) / MAX_RPM * 100, MAX_SPEED)
        print(f"  Motor {i+1}: {motor_rpms[i]:.2f} RPM, PWM = {duty:.1f}%")
        set_motor_pwm(*motor_pins[i], motor_rpms[i])

def stop_motors():
    for pin in [M1_ENA, M2_ENA, M3_ENA]:
        lgpio.tx_pwm(h, pin, PWM_FREQUENCY, 0)

def get_input(prompt, default, cast_type=float):
    try:
        val = input(f"{prompt} [{default}]: ")
        return cast_type(val) if val.strip() != "" else default
    except ValueError:
        print("Invalid input. Using default.")
        return default

def user_input():
    print("\n--- Delta Robot Control Interface ---")
    vel_ref = get_input("Enter linear speed [m/s]", 0.3)
    theta_deg = get_input("Enter platform direction [deg]", 0.0)
    omega_z_ref = get_input("Enter angular speed [rad/s]", 0.0)
    duration = get_input("Enter movement duration [s]", 5.0)
    return vel_ref, math.radians(theta_deg), omega_z_ref, duration

try:
    while True:
        vel_ref, theta_ref, omega_z_ref, duration = user_input()
        motor_rpms = compute_motor_rpms(vel_ref, theta_ref, omega_z_ref)
        drive_motors(motor_rpms)
        print("\nRunning motors...")
        time.sleep(duration)
        stop_motors()
        print("Motion complete. Press Ctrl+C to exit or input new command.")
except KeyboardInterrupt:
    print("\nStopping all motors.")
    stop_motors()
    lgpio.gpiochip_close(h)
