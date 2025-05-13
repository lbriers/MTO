# -*- coding: utf-8 -*-
import time
import math
import board
import adafruit_lsm9ds1
import numpy as np
import sys
import collections
import csv
import os
from datetime import datetime
# Import Madgwick library directly
from madgwick_py.quaternion import Quaternion
from madgwick_py.madgwickahrs import MadgwickAHRS

# Create sensor object
i2c = board.I2C()
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Initialize Madgwick filter
sample_period = 0.001  
# Beta value (algorithm gain) - adjust as needed for responsiveness vs. smoothing
madgwick = MadgwickAHRS(sampleperiod=sample_period, beta=5)

# Magnetometer calibration parameters
mag_offset = (0.4774, 0.0024, 0.0079)  # calibrated offsets (bias)
mag_scale = (0.9514, 0.9809, 1.0760)   # Calibrated scaling factors

# Moving average filter parameters
# Define window size for different sensor types
ACCEL_WINDOW_SIZE = 35
GYRO_WINDOW_SIZE = 35
MAG_WINDOW_SIZE = 50
ORIENTATION_WINDOW_SIZE = 100

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
def trimmed_mean(buffer, trim_percent=0.2):
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

# Create a unique filename for the CSV based on current date and time
def create_csv_file():
    # Create 'data' directory if it doesn't exist
    data_dir = 'data'
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)
    
    # Create filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(data_dir, f'imu_data_{timestamp}.csv')
    
    # Create and prepare the CSV file with headers
    with open(filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
                    # Write header row
        csv_writer.writerow([
            'timestamp',
            # Raw accelerometer data
            'accel_x_raw', 'accel_y_raw', 'accel_z_raw',
            # Filtered accelerometer data
            'accel_x_filtered', 'accel_y_filtered', 'accel_z_filtered',
            # Raw magnetometer data
            'mag_x_raw', 'mag_y_raw', 'mag_z_raw',
            # Filtered magnetometer data
            'mag_x_filtered', 'mag_y_filtered', 'mag_z_filtered',
            # Normalized magnetometer data
            'mag_x_norm', 'mag_y_norm', 'mag_z_norm',
            # Raw gyroscope data
            'gyro_x_raw', 'gyro_y_raw', 'gyro_z_raw',
            # Filtered gyroscope data
            'gyro_x_filtered', 'gyro_y_filtered', 'gyro_z_filtered',
            # Magnetic magnitude
            'mag_magnitude',
            # Quaternion components (from Madgwick filter)
            'quat_w', 'quat_x', 'quat_y', 'quat_z',
            # Raw Euler angles
            'roll_raw', 'pitch_raw', 'yaw_raw',
            # Filtered Euler angles
            'roll_filtered', 'pitch_filtered', 'yaw_filtered',
            # Raw quaternion data (unfiltered) - derived from raw gyro/accel/mag data
            'quat_raw_w', 'quat_raw_x', 'quat_raw_y', 'quat_raw_z',
            # Unfiltered Euler angles - direct from raw quaternion
            'roll_unfiltered', 'pitch_unfiltered', 'yaw_unfiltered'
        ])
    
    print(f"Created CSV file: {filename}")
    return filename

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
    
    # Create CSV file for data logging
    csv_filename = create_csv_file()
    
    # Start time for timestamp calculation
    start_time = time.time()
    
    # Set how often to save data (every N iterations)
    save_interval = 5  # Adjust this based on your desired save frequency
    iteration_count = 0
    
    # Create a separate Madgwick filter instance for raw data (no moving average)
    madgwick_raw = MadgwickAHRS(sampleperiod=sample_period, beta=5)
    
    while True:
        # Increment iteration counter
        iteration_count += 1
        
        # Calculate timestamp (seconds since start)
        current_time = time.time() - start_time
        
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
        
        # Calculate magnitude of raw magnetic vector
        raw_magnitude = math.sqrt(mag_x**2 + mag_y**2 + mag_z**2)
        
        # Normalize raw magnetometer data
        if raw_magnitude > 0:
            raw_mag_x_norm = mag_x/raw_magnitude
            raw_mag_y_norm = mag_y/raw_magnitude
            raw_mag_z_norm = mag_z/raw_magnitude
        else:
            raw_mag_x_norm, raw_mag_y_norm, raw_mag_z_norm = 0, 0, 0
        
        # Update raw Madgwick filter using unfiltered data
        madgwick_raw.update([gyro_x, gyro_y, gyro_z], 
                          [accel_x, accel_y, accel_z], 
                          [raw_mag_x_norm, raw_mag_y_norm, raw_mag_z_norm])
        
        # Get raw quaternion (unfiltered)
        q_raw = madgwick_raw.quaternion
        
        # Convert raw quaternion to unfiltered Euler angles
        roll_unfiltered, pitch_unfiltered, yaw_unfiltered = quaternion_to_euler(q_raw)
        
        # Add orientation data to the moving average buffers
        roll_buffer.append(roll)
        pitch_buffer.append(pitch)
        yaw_buffer.append(yaw)
        
        # Calculate trimmed mean for orientation data
        roll_avg = trimmed_mean(roll_buffer)
        pitch_avg = trimmed_mean(pitch_buffer)
        yaw_avg = trimmed_mean(yaw_buffer)
        
        # Save data to CSV file (at specified interval to avoid too frequent disk writes)
        if iteration_count % save_interval == 0:
            with open(csv_filename, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([
                    f"{current_time:.3f}",
                    # Raw accelerometer data
                    f"{accel_x:.6f}", f"{accel_y:.6f}", f"{accel_z:.6f}",
                    # Filtered accelerometer data
                    f"{accel_x_avg:.6f}", f"{accel_y_avg:.6f}", f"{accel_z_avg:.6f}",
                    # Raw magnetometer data
                    f"{mag_x:.6f}", f"{mag_y:.6f}", f"{mag_z:.6f}",
                    # Filtered magnetometer data
                    f"{mag_x_avg:.6f}", f"{mag_y_avg:.6f}", f"{mag_z_avg:.6f}",
                    # Normalized magnetometer data
                    f"{mag_x_norm:.6f}", f"{mag_y_norm:.6f}", f"{mag_z_norm:.6f}",
                    # Raw gyroscope data
                    f"{gyro_x:.6f}", f"{gyro_y:.6f}", f"{gyro_z:.6f}",
                    # Filtered gyroscope data
                    f"{gyro_x_avg:.6f}", f"{gyro_y_avg:.6f}", f"{gyro_z_avg:.6f}",
                    # Magnetic magnitude
                    f"{magnitude:.6f}",
                    # Quaternion components (filtered)
                    f"{q[0]:.6f}", f"{q[1]:.6f}", f"{q[2]:.6f}", f"{q[3]:.6f}",
                    # Raw Euler angles
                    f"{roll:.6f}", f"{pitch:.6f}", f"{yaw:.6f}",
                    # Filtered Euler angles
                    f"{roll_avg:.6f}", f"{pitch_avg:.6f}", f"{yaw_avg:.6f}",
                    # Raw quaternion data (unfiltered)
                    f"{q_raw[0]:.6f}", f"{q_raw[1]:.6f}", f"{q_raw[2]:.6f}", f"{q_raw[3]:.6f}",
                    # Unfiltered Euler angles
                    f"{roll_unfiltered:.6f}", f"{pitch_unfiltered:.6f}", f"{yaw_unfiltered:.6f}"
                ])
        
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
        print(f"Filtered Quaternion: W={q[0]:.3f}, X={q[1]:.3f}, Y={q[2]:.3f}, Z={q[3]:.3f}")
        print(f"Raw Quaternion: W={q_raw[0]:.3f}, X={q_raw[1]:.3f}, Y={q_raw[2]:.3f}, Z={q_raw[3]:.3f}")
        print(f"Raw Euler angles: Roll={roll:.1f} deg, Pitch={pitch:.1f} deg, Yaw={yaw:.1f} deg")
        print(f"Filtered Euler angles: Roll={roll_avg:.1f} deg, Pitch={pitch_avg:.1f} deg, Yaw={yaw_avg:.1f} deg")
        print(f"Unfiltered Euler angles: Roll={roll_unfiltered:.1f} deg, Pitch={pitch_unfiltered:.1f} deg, Yaw={yaw_unfiltered:.1f} deg")
        print(f"Data saved to: {csv_filename}")
        print("-" * 50)
        
        time.sleep(sample_period)
        
except KeyboardInterrupt:
    print("Program terminated by user")
    print(f"Data saved to: {csv_filename}")