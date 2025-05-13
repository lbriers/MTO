# -*- coding: utf-8 -*-
import time
import math
import board
import adafruit_lsm9ds1
import numpy as np
import sys
import collections
# Import Madgwick library directly
from madgwick_py.quaternion import Quaternion
from madgwick_py.madgwickahrs import MadgwickAHRS

# Create sensor object
i2c = board.I2C()
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Initialize Madgwick filter
sample_period = 0.001  
# Beta value (algorithm gain) - adjust as needed for responsiveness vs. smoothing
madgwick = MadgwickAHRS(sampleperiod=sample_period, beta=10)

# Magnetometer calibration parameters
mag_offset = (0.4774, 0.0024, 0.0079)  # calibrated offsets (bias)
mag_scale = (0.9514, 0.9809, 1.0760)   # Calibrated scaling factors

# Moving average filter parameters
# Define window size for different sensor types
ACCEL_WINDOW_SIZE = 10
GYRO_WINDOW_SIZE = 8
MAG_WINDOW_SIZE = 15
ORIENTATION_WINDOW_SIZE = 12

# Initialize deques for moving averages
accel_buffer_x = collections.deque(maxlen=ACCEL_WINDOW_SIZE)
accel_buffer_y = collections.deque(maxlen=ACCEL_WINDOW_SIZE)
accel_buffer_z = collections.deque(maxlen=ACCEL_WINDOW_SIZE)

gyro_buffer_x = collections.deque(maxlen=GYRO_WINDOW_SIZE)
gyro_buffer_y = collections.deque(maxlen=GYRO_WINDOW_SIZE)
gyro_buffer_z = collections.deque(maxlen=GYRO_WINDOW_SIZE)

mag_buffer_x = collections.deque(maxlen=MAG_WINDOW_SIZE)
mag_buffer_y = collections.deque(maxlen=MAG_WINDOW_SIZE)
mag_buffer_z = collections.deque(maxlen=MAG_WINDOW_SIZE)

roll_buffer = collections.deque(maxlen=ORIENTATION_WINDOW_SIZE)
pitch_buffer = collections.deque(maxlen=ORIENTATION_WINDOW_SIZE)
yaw_buffer = collections.deque(maxlen=ORIENTATION_WINDOW_SIZE)

# Function to apply trimmed mean to a buffer
# Removes 10% from the lowest and 10% from the highest values
def trimmed_mean(buffer, trim_percent=0.1):
    if len(buffer) == 0:
        return 0
    
    # Convert to list and sort
    sorted_values = sorted(buffer)
    
    # Calculate how many items to remove from each end
    trim_count = int(len(sorted_values) * trim_percent)
    
    # If the buffer is too small, just return regular average
    if len(sorted_values) <= 2:
        return sum(sorted_values) / len(sorted_values)
    
    # Remove the lowest and highest values based on trim_count
    trimmed_values = sorted_values[trim_count:len(sorted_values)-trim_count]
    
    # Calculate the mean of the remaining values
    if len(trimmed_values) == 0:  # Safety check
        return sum(sorted_values) / len(sorted_values)
    
    return sum(trimmed_values) / len(trimmed_values)

# Function to calculate Euler angles from quaternion
def quaternion_to_euler(q):
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (q[0] * q[2] - q[3] * q[1])
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
    cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # Convert to degrees
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)
    
    # Normalize yaw to 0-360
    if yaw_deg < 0:
        yaw_deg += 360
        
    return roll_deg, pitch_deg, yaw_deg

# Function to apply magnetometer calibration
def calibrate_mag(raw_mag):
    mx, my, mz = raw_mag
    # Apply offset correction (bias)
    mx -= mag_offset[0]
    my -= mag_offset[1]
    mz -= mag_offset[2]
    
    # Apply scale correction
    mx *= mag_scale[0]
    my *= mag_scale[1]
    mz *= mag_scale[2]
    
    return (mx, my, mz)

# Main loop
try:
    # Short initialization - just a couple of samples
    print("Initializing sensor...")
    for _ in range(5):  # Just a few samples
        sensor.acceleration
        sensor.magnetic
        sensor.gyro
        time.sleep(0.01)
    print("Initialization complete!")
    
    while True:
        # Read sensor data
        accel_x, accel_y, accel_z = sensor.acceleration
        accel_x, accel_y, accel_z = -accel_x, -accel_y, -accel_z  # Original inversion
        
        # Add accelerometer data to the moving average buffers
        accel_buffer_x.append(accel_x)
        accel_buffer_y.append(accel_y)
        accel_buffer_z.append(accel_z)
        
        # Calculate trimmed mean for accelerometer data
        accel_x_avg = trimmed_mean(accel_buffer_x)
        accel_y_avg = trimmed_mean(accel_buffer_y)
        accel_z_avg = trimmed_mean(accel_buffer_z)
        
        # Get magnetometer data
        mag_x, mag_y, mag_z = sensor.magnetic
        
        # Apply basic calibration to magnetometer data
        mag_x, mag_y, mag_z = calibrate_mag((mag_x, mag_y, mag_z))
        
        # Add magnetometer data to the moving average buffers
        mag_buffer_x.append(mag_x)
        mag_buffer_y.append(mag_y)
        mag_buffer_z.append(mag_z)
        
        # Calculate trimmed mean for magnetometer data
        mag_x_avg = trimmed_mean(mag_buffer_x)
        mag_y_avg = trimmed_mean(mag_buffer_y)
        mag_z_avg = trimmed_mean(mag_buffer_z)
        
        # Get gyroscope data
        gyro_x, gyro_y, gyro_z = sensor.gyro
        
        # Add gyroscope data to the moving average buffers
        gyro_buffer_x.append(gyro_x)
        gyro_buffer_y.append(gyro_y)
        gyro_buffer_z.append(gyro_z)
        
        # Calculate trimmed mean for gyroscope data
        gyro_x_avg = trimmed_mean(gyro_buffer_x)
        gyro_y_avg = trimmed_mean(gyro_buffer_y)
        gyro_z_avg = trimmed_mean(gyro_buffer_z)
        
        # Calculate magnitude of magnetic vector using averaged data
        magnitude = math.sqrt(mag_x_avg**2 + mag_y_avg**2 + mag_z_avg**2)
        
        # Normalize magnetometer data
        if magnitude > 0:
            mag_x_norm = mag_x_avg/magnitude
            mag_y_norm = mag_y_avg/magnitude
            mag_z_norm = mag_z_avg/magnitude
        else:
            mag_x_norm, mag_y_norm, mag_z_norm = 0, 0, 0
        
        # Update Madgwick filter using smoothed data
        madgwick.update([gyro_x_avg, gyro_y_avg, gyro_z_avg], 
                        [accel_x_avg, accel_y_avg, accel_z_avg], 
                        [mag_x_norm, mag_y_norm, mag_z_norm])
        
        # Get quaternion from Madgwick filter
        q = madgwick.quaternion
        
        # Convert quaternion to Euler angles
        roll, pitch, yaw = quaternion_to_euler(q)
        
        # Add orientation data to the moving average buffers
        roll_buffer.append(roll)
        pitch_buffer.append(pitch)
        yaw_buffer.append(yaw)
        
        # Calculate trimmed mean for orientation data
        roll_avg = trimmed_mean(roll_buffer)
        pitch_avg = trimmed_mean(pitch_buffer)
        yaw_avg = trimmed_mean(yaw_buffer)
        
        # Print raw sensor data
        print("=== Raw Sensor Data ===")
        print(f"Accelerometer (m/s^2): X={accel_x:.3f}, Y={accel_y:.3f}, Z={accel_z:.3f}")
        print(f"Magnetometer (gauss): X={mag_x:.3f}, Y={mag_y:.3f}, Z={mag_z:.3f}")
        print(f"Gyroscope (rad/s): X={gyro_x:.3f}, Y={gyro_y:.3f}, Z={gyro_z:.3f}")
        
        # Print filtered sensor data
        print("\n=== Filtered Sensor Data ===")
        print(f"Accelerometer (m/s^2): X={accel_x_avg:.3f}, Y={accel_y_avg:.3f}, Z={accel_z_avg:.3f}")
        print(f"Magnetometer (gauss): X={mag_x_avg:.3f}, Y={mag_y_avg:.3f}, Z={mag_z_avg:.3f}")
        print(f"Normalized Mag: X={mag_x_norm:.3f}, Y={mag_y_norm:.3f}, Z={mag_z_norm:.3f}")
        print(f"Gyroscope (rad/s): X={gyro_x_avg:.3f}, Y={gyro_y_avg:.3f}, Z={gyro_z_avg:.3f}")
        print(f"Magnetic magnitude: {magnitude:.3f} gauss")
        
        # Print orientation data
        print("\n=== Orientation Data ===")
        print(f"Quaternion: W={q[0]:.3f}, X={q[1]:.3f}, Y={q[2]:.3f}, Z={q[3]:.3f}")
        print(f"Raw Euler angles: Roll={roll:.1f} deg, Pitch={pitch:.1f} deg, Yaw={yaw:.1f} deg")
        print(f"Filtered Euler angles: Roll={roll_avg:.1f} deg, Pitch={pitch_avg:.1f} deg, Yaw={yaw_avg:.1f} deg")
        print("-" * 50)
        
        time.sleep(sample_period)
        
except KeyboardInterrupt:
    print("Program terminated by user")