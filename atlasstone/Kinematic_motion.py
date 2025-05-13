# -*- coding: utf-8 -*-
import time
import math
import lgpio

# Motor pin definitions
M1_IN1 = 22
M1_IN2 = 23
M1_ENA = 13
M2_IN1 = 24
M2_IN2 = 25
M2_ENA = 18
M3_IN1 = 17
M3_IN2 = 27
M3_ENA = 12

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
RAD_S_TO_RPM = 60 / (2 * math.pi)
RPM_TO_RAD_S = (2 * math.pi) / 60
MAX_RPM = 188  # theoretical max motor speed in RPM

# GPIO setup
h = lgpio.gpiochip_open(0)
pins = [M1_IN1, M1_IN2, M1_ENA, M2_IN1, M2_IN2, M2_ENA, M3_IN1, M3_IN2, M3_ENA]
for pin in pins:
    lgpio.gpio_claim_output(h, pin, 0)

def rpm_to_pwm(rpm):
    """Convert desired RPM to PWM duty using empirical 4th-degree polynomial."""
    pwm = (-1.34e-6 * rpm**4
           + 5.1137e-4 * rpm**3
           - 0.06308958 * rpm**2
           + 3.28124028 * rpm
           - 26.41387171)
    return max(0, min(pwm, MAX_SPEED))  # Clamp to 0-100%

def compute_motor_rpms(vel_ref, theta_ref, omega_z_ref):
    """Compute target RPM for each motor from linear and angular velocities."""
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
    """Set motor direction and apply PWM signal based on desired RPM."""
    direction = 1 if motor_rpm >= 0 else 0
    lgpio.gpio_write(h, pin_in1, direction)
    lgpio.gpio_write(h, pin_in2, 1 - direction)
    duty_cycle = rpm_to_pwm(abs(motor_rpm))
    lgpio.tx_pwm(h, pin_ena, PWM_FREQUENCY, duty_cycle)

def drive_motors(motor_rpms):
    """Drive all motors based on RPM values."""
    motor_pins = [
        (M1_IN1, M1_IN2, M1_ENA),
        (M2_IN1, M2_IN2, M2_ENA),
        (M3_IN1, M3_IN2, M3_ENA)
    ]
    print("\nMotor Commands:")
    for i in range(3):
        pwm = rpm_to_pwm(abs(motor_rpms[i]))
        print(f"  Motor {i+1}: {motor_rpms[i]:.2f} RPM -> PWM = {pwm:.1f}%")
        set_motor_pwm(*motor_pins[i], motor_rpms[i])

def stop_motors():
    """Stop all motors by setting PWM to zero."""
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
