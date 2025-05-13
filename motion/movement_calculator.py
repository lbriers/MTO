# -*- coding: utf-8 -*-
import time
import math

# Delta platform geometry (in meters)
r_w = 0.03   # wheel radius = 30 mm
r_b = 0.15   # base link offset = 150 mm

# Platform link angles in radians
theta_1 = 0
theta_2 = -120 * math.pi / 180
theta_3 = -240 * math.pi / 180  # equivalent to +120° in unit circle

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

def get_float(prompt, default=0.0):
    """Helper function to get float input with a default value."""
    try:
        return float(input(prompt))
    except ValueError:
        return default

# Main interactive loop
print("Delta Platform Motor Speed Simulator\n")
print("Enter values for linear velocity [m/s], orientation [deg], and angular velocity [rad/s].")
print("Press Ctrl+C or type 'q' at any time to quit.\n")

try:
    while True:
        user_input = input("Continue (Enter to proceed, 'q' to quit): ")
        if user_input.lower() == 'q':
            break

        vel_ref = get_float("Linear velocity (m/s) [e.g., 0.05]: ", 0.05)
        theta2_deg = get_float("Platform orientation (degrees) [e.g., 0]: ", 0)
        omega_z_ref = get_float("Angular velocity (rad/s) [e.g., 0.2]: ", 0.2)

        theta2_ref = math.radians(theta2_deg)

        motor_speeds = compute_motor_speeds(vel_ref, theta2_ref, omega_z_ref)

        print(f"\nMotor Speeds (rad/s):")
        for i, omega in enumerate(motor_speeds, start=1):
            print(f"  Motor {i}: {omega:.3f} rad/s")
        print()

except KeyboardInterrupt:
    print("\nSimulation stopped.")

