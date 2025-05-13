# -*- coding: utf-8 -*-
import time
import math
import board
import adafruit_lsm9ds1
import numpy as np
import lgpio

from madgwick_py.quaternion import Quaternion
from madgwick_py.madgwickahrs import MadgwickAHRS

# Motor pin definitions
M1_IN1 = 17
M1_IN2 = 27
M1_ENA = 12
M2_IN1 = 22
M2_IN2 = 23
M2_ENA = 13
M3_IN1 = 24
M3_IN2 = 25
M3_ENA = 18

PWM_FREQUENCY = 100  # Hz
MAX_SPEED = 100

# Delta platform geometry (in meters)
r_w = 0.03
r_b = 0.15

# Platform wheel angles
theta_1 = 0
theta_2 = 120 * math.pi / 180
theta_3 = 240 * math.pi / 180

# Motor control setup
h = lgpio.gpiochip_open(0)
motor_pins = [(M1_IN1, M1_IN2, M1_ENA),
              (M2_IN1, M2_IN2, M2_ENA),
              (M3_IN1, M3_IN2, M3_ENA)]

for pin_group in motor_pins:
    for pin in pin_group:
        lgpio.gpio_claim_output(h, pin, 0)

# IMU setup
i2c = board.I2C()
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
madgwick = MadgwickAHRS(sampleperiod=0.001, beta=1)

def quaternion_to_euler(q):
    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1 - 2 * (q[1]**2 + q[2]**2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q[0] * q[2] - q[3] * q[1])
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
    cosy_cosp = 1 - 2 * (q[2]**2 + q[3]**2)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw) % 360

def compute_motor_speeds(vel_ref, theta_ref_rad, omega_z_ref):
    speeds = []
    for theta_i in [theta_1, theta_2, theta_3]:
        linear = vel_ref * math.cos(theta_i - theta_ref_rad)
        angular = omega_z_ref * r_b
        omega_i = (1 / r_w) * (linear + angular)
        speeds.append(omega_i)
    return speeds

def set_motor_pwm(pin_in1, pin_in2, pin_ena, speed):
    max_rad_per_sec = 10.0
    duty = min(abs(speed) / max_rad_per_sec * 100, MAX_SPEED)
    direction = 1 if speed >= 0 else 0
    lgpio.gpio_write(h, pin_in1, direction)
    lgpio.gpio_write(h, pin_in2, 1 - direction)
    lgpio.tx_pwm(h, pin_ena, PWM_FREQUENCY, duty)
    return duty

def stop_motors():
    for _, _, ena in motor_pins:
        lgpio.tx_pwm(h, ena, PWM_FREQUENCY, 0)

try:
    print("Starting IMU-based Delta Robot Control (press Ctrl+C to stop)...")
    vel_ref = 0.1     # m/s forward speed
    omega_z_ref = 0.0 # rad/s rotation

    while True:
        # Read and normalize IMU data
        ax, ay, az = sensor.acceleration
        gx, gy, gz = sensor.gyro
        mx, my, mz = sensor.magnetic
        ax, ay, az = -ax, -ay, -az
        mag_norm = math.sqrt(mx**2 + my**2 + mz**2)
        mx, my, mz = mx/mag_norm, my/mag_norm, mz/mag_norm

        # Update Madgwick filter
        madgwick.update([gx, gy, gz], [ax, ay, az], [mx, my, mz])
        q = madgwick.quaternion
        _, _, yaw = quaternion_to_euler(q)

        theta_ref_rad = math.radians(yaw)
        speeds = compute_motor_speeds(vel_ref, theta_ref_rad, omega_z_ref)

        print(f"\nYaw: {yaw:.1f}deg -> theta_ref = {math.degrees(theta_ref_rad):.1f}deg")
        print("Motor commands:")

        for i, (pins, omega) in enumerate(zip(motor_pins, speeds)):
            duty = set_motor_pwm(*pins, omega)
            print(f"  Motor {i+1}: omega = {omega:.2f} rad/s, PWM = {duty:.1f}%")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping motors and exiting...")
    stop_motors()
    lgpio.gpiochip_close(h)
