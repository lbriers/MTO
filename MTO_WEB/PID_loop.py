# -*- coding: utf-8 -*-
import time
import math
import board
import adafruit_lsm9ds1
import numpy as np
import sys
import collections
import threading
import queue
import lgpio
import http.server
import socketserver
import json
import logging
from simple_pid import PID

# Logging setup
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

website_data = {"alpha": 0, "beta": 0, "gamma": 0}
website_lock = threading.Lock()

# Motor pin definitions
M1_IN1, M1_IN2, M1_ENA = 22, 23, 13
M2_IN1, M2_IN2, M2_ENA = 24, 25, 18
M3_IN1, M3_IN2, M3_ENA = 17, 27, 12

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
MAX_SAFE_RPM = 174 # safe max motor speed in RPM

# PWM and speed settings
PWM_FREQUENCY = 100  # Hz
MAX_SPEED = 100      # Max PWM duty cycle

# Shared state and synchronization
class SharedState:
    def __init__(self):
        self.target_roll = 0
        self.target_pitch = 0
        self.target_yaw = 0
        self.current_roll = 0
        self.current_pitch = 0
        self.current_yaw = 0
        self.lock = threading.Lock()

# Import Madgwick library directly
from madgwick_py.quaternion import Quaternion
from madgwick_py.madgwickahrs import MadgwickAHRS

class AtlasstoneController:
    def __init__(self):
        # Shared state
        self.shared_state = SharedState()
        
        # HTTP server queue for incoming commands
        self.command_queue = queue.Queue()
        
        # GPIO and motor setup
        self.h = lgpio.gpiochip_open(0)
        pins = [M1_IN1, M1_IN2, M1_ENA, M2_IN1, M2_IN2, M2_ENA, M3_IN1, M3_IN2, M3_ENA]
        for pin in pins:
            lgpio.gpio_claim_output(self.h, pin, 0)
        
        # PID Controllers for Roll, Pitch, Yaw
        # Set sample time explicitly to prevent dt calculation issues
        self.pid_sample_time = 0.1  # 10 Hz update rate
        
        # Initialize PID controllers with explicit sample time
        self.pid_roll = PID(0.01, 0.01, 0.005, 
                            setpoint=0, 
                            sample_time=self.pid_sample_time,
                            output_limits=(-MAX_SAFE_RPM, MAX_SAFE_RPM))
        self.pid_pitch = PID(0.01, 0.01, 0.005, 
                             setpoint=0, 
                             sample_time=self.pid_sample_time,
                             output_limits=(-MAX_SAFE_RPM, MAX_SAFE_RPM))
        # Reduced PID values for yaw to make it less aggressive
        self.pid_yaw = PID(0.1, 0.01, 0.0005, 
                           setpoint=0, 
                           sample_time=self.pid_sample_time,
                           output_limits=(-MAX_SAFE_RPM/4, MAX_SAFE_RPM/4))
        
        # Sensor and filter initialization
        self.init_sensor_components()
        
    def init_sensor_components(self):
        # Create sensor object
        i2c = board.I2C()
        self.sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

        # Initialize Madgwick filter
        self.sample_period = 0.001  
        self.madgwick = MadgwickAHRS(sampleperiod=self.sample_period, beta=5)

        # Magnetometer calibration parameters
        self.mag_offset = (0.4774, 0.0024, 0.0079)  # calibrated offsets (bias)
        self.mag_scale = (0.9514, 0.9809, 1.0760)   # Calibrated scaling factors

        # Moving average filter parameters
        self.ACCEL_WINDOW_SIZE = 10
        self.GYRO_WINDOW_SIZE = 8
        self.MAG_WINDOW_SIZE = 15
        self.ORIENTATION_WINDOW_SIZE = 20

        # Initialize buffers
        self.init_moving_average_buffers()
        
    def init_moving_average_buffers(self):
        """Initialize deques for moving averages."""
        self.accel_buffer_x = collections.deque(maxlen=self.ACCEL_WINDOW_SIZE)
        self.accel_buffer_y = collections.deque(maxlen=self.ACCEL_WINDOW_SIZE)
        self.accel_buffer_z = collections.deque(maxlen=self.ACCEL_WINDOW_SIZE)

        self.gyro_buffer_x = collections.deque(maxlen=self.GYRO_WINDOW_SIZE)
        self.gyro_buffer_y = collections.deque(maxlen=self.GYRO_WINDOW_SIZE)
        self.gyro_buffer_z = collections.deque(maxlen=self.GYRO_WINDOW_SIZE)

        self.mag_buffer_x = collections.deque(maxlen=self.MAG_WINDOW_SIZE)
        self.mag_buffer_y = collections.deque(maxlen=self.MAG_WINDOW_SIZE)
        self.mag_buffer_z = collections.deque(maxlen=self.MAG_WINDOW_SIZE)

        self.roll_buffer = collections.deque(maxlen=self.ORIENTATION_WINDOW_SIZE)
        self.pitch_buffer = collections.deque(maxlen=self.ORIENTATION_WINDOW_SIZE)
        self.yaw_buffer = collections.deque(maxlen=self.ORIENTATION_WINDOW_SIZE)
        
    def normalize_angle_error(self, current, target):
        """
        Calculate the shortest angle difference between current and target angles.
        Takes into account the circular nature of angles (0 and 360 are the same).
        
        Args:
            current: Current angle in degrees (0-360)
            target: Target angle in degrees (0-360)
        
        Returns:
            Angle error in degrees (-180 to 180)
        """
        # Ensure both angles are in the range 0-360
        current = current % 360
        target = target % 360
        
        # Calculate the direct difference
        error = target - current
        
        # Adjust for the shortest path
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        return error
        
    def trimmed_mean(self, buffer, trim_percent=0.1):
        """Calculate trimmed mean of a buffer."""
        if len(buffer) == 0:
            return 0
        
        sorted_values = sorted(buffer)
        trim_count = int(len(sorted_values) * trim_percent)
        
        if len(sorted_values) <= 2:
            return sum(sorted_values) / len(sorted_values)
        
        trimmed_values = sorted_values[trim_count:len(sorted_values)-trim_count]
        
        if len(trimmed_values) == 0:
            return sum(sorted_values) / len(sorted_values)
        
        return sum(trimmed_values) / len(trimmed_values)
    
    def calibrate_mag(self, raw_mag):
        """Apply magnetometer calibration."""
        mx, my, mz = raw_mag
        mx -= self.mag_offset[0]
        my -= self.mag_offset[1]
        mz -= self.mag_offset[2]
        
        mx *= self.mag_scale[0]
        my *= self.mag_scale[1]
        mz *= self.mag_scale[2]
        
        return (mx, my, mz)
    
    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles."""
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        if yaw_deg < 0:
            yaw_deg += 360
            
        return roll_deg, pitch_deg, yaw_deg
    
    def rpm_to_pwm(self, rpm):
        """Convert desired RPM to PWM duty using empirical 4th-degree polynomial."""
        pwm = (-1.34e-6 * rpm**4
               + 5.1137e-4 * rpm**3
               - 0.06308958 * rpm**2
               + 3.28124028 * rpm
               - 26.41387171)
        return max(0, min(pwm, MAX_SPEED))
    
    def compute_motor_rpms(self, vel_ref, theta_ref, omega_z_ref):
        """Compute target RPM for each motor from linear and angular velocities."""
        motor_rpms = []
        angles = [theta_1, theta_2, theta_3]
        for angle in angles:
            linear_component = vel_ref * math.cos(angle - theta_ref)
            angular_component = omega_z_ref * r_b
            wheel_omega_rad = (1 / r_w) * (linear_component + angular_component)
            wheel_omega_rpm = wheel_omega_rad * RAD_S_TO_RPM
            # Limit RPM to safe range
            wheel_omega_rpm = max(-MAX_SAFE_RPM, min(wheel_omega_rpm, MAX_SAFE_RPM))
            motor_rpms.append(wheel_omega_rpm)
        return motor_rpms
    
    def set_motor_pwm(self, pin_in1, pin_in2, pin_ena, motor_rpm):
        """Set motor direction and apply PWM signal based on desired RPM."""
        direction = 1 if motor_rpm >= 0 else 0
        lgpio.gpio_write(self.h, pin_in1, direction)
        lgpio.gpio_write(self.h, pin_in2, 1 - direction)
        duty_cycle = self.rpm_to_pwm(abs(motor_rpm))
        lgpio.tx_pwm(self.h, pin_ena, PWM_FREQUENCY, duty_cycle)
    
    def drive_motors(self, motor_rpms):
        """Drive all motors based on RPM values."""
        motor_pins = [
            (M1_IN1, M1_IN2, M1_ENA),
            (M2_IN1, M2_IN2, M2_ENA),
            (M3_IN1, M3_IN2, M3_ENA)
        ]
        logger.info("\nMotor Commands:")
        for i in range(3):
            pwm = self.rpm_to_pwm(abs(motor_rpms[i]))
            logger.info(f"  Motor {i+1}: {motor_rpms[i]:.2f} RPM -> PWM = {pwm:.1f}%")
            self.set_motor_pwm(*motor_pins[i], motor_rpms[i])
    
    def stop_motors(self):
        """Stop all motors by setting PWM to zero."""
        for pin in [M1_ENA, M2_ENA, M3_ENA]:
            lgpio.tx_pwm(self.h, pin, PWM_FREQUENCY, 0)
    
    def imu_thread_func(self):
        """Thread function for reading and processing IMU data."""
        try:
            # Short initialization
            logger.info("Initializing IMU sensor...")
            for _ in range(5):
                self.sensor.acceleration
                self.sensor.magnetic
                self.sensor.gyro
                time.sleep(0.01)
            logger.info("IMU initialization complete!")
            
            while True:
                # Read and process sensor data
                accel_x, accel_y, accel_z = self.sensor.acceleration
                accel_x, accel_y, accel_z = -accel_x, -accel_y, -accel_z
                
                # Accelerometer moving average
                self.accel_buffer_x.append(accel_x)
                self.accel_buffer_y.append(accel_y)
                self.accel_buffer_z.append(accel_z)
                
                accel_x_avg = self.trimmed_mean(self.accel_buffer_x)
                accel_y_avg = self.trimmed_mean(self.accel_buffer_y)
                accel_z_avg = self.trimmed_mean(self.accel_buffer_z)
                
                # Magnetometer data
                mag_x, mag_y, mag_z = self.sensor.magnetic
                mag_x, mag_y, mag_z = self.calibrate_mag((mag_x, mag_y, mag_z))
                
                self.mag_buffer_x.append(mag_x)
                self.mag_buffer_y.append(mag_y)
                self.mag_buffer_z.append(mag_z)
                
                mag_x_avg = self.trimmed_mean(self.mag_buffer_x)
                mag_y_avg = self.trimmed_mean(self.mag_buffer_y)
                mag_z_avg = self.trimmed_mean(self.mag_buffer_z)
                
                # Normalize magnetometer data
                magnitude = math.sqrt(mag_x_avg**2 + mag_y_avg**2 + mag_z_avg**2)
                
                if magnitude > 0:
                    mag_x_norm = mag_x_avg/magnitude
                    mag_y_norm = mag_y_avg/magnitude
                    mag_z_norm = mag_z_avg/magnitude
                else:
                    mag_x_norm, mag_y_norm, mag_z_norm = 0, 0, 0
                
                # Gyroscope data
                gyro_x, gyro_y, gyro_z = self.sensor.gyro
                
                self.gyro_buffer_x.append(gyro_x)
                self.gyro_buffer_y.append(gyro_y)
                self.gyro_buffer_z.append(gyro_z)
                
                gyro_x_avg = self.trimmed_mean(self.gyro_buffer_x)
                gyro_y_avg = self.trimmed_mean(self.gyro_buffer_y)
                gyro_z_avg = self.trimmed_mean(self.gyro_buffer_z)
                
                # Update Madgwick filter
                self.madgwick.update(
                    [gyro_x_avg, gyro_y_avg, gyro_z_avg], 
                    [accel_x_avg, accel_y_avg, accel_z_avg], 
                    [mag_x_norm, mag_y_norm, mag_z_norm]
                )
                
                # Get quaternion and convert to Euler angles
                q = self.madgwick.quaternion
                roll, pitch, yaw = self.quaternion_to_euler(q)
                
                # Update shared state with current orientation
                with self.shared_state.lock:
                    self.shared_state.current_roll = roll
                    self.shared_state.current_pitch = pitch
                    self.shared_state.current_yaw = yaw
                
                time.sleep(self.sample_period)
        except Exception as e:
            logger.error(f"Error in IMU thread: {e}")
        finally:
            logger.info("IMU Thread Terminated")
    
        
    def motor_control_thread_func(self):
        """Thread function for PID-based motor control."""
        last_update_time = time.time()
        try:
            while True:
                # Calculate time since last update
                current_time = time.time()
                dt = current_time - last_update_time
                last_update_time = current_time
                
                # Check if time difference is valid
                if dt <= 0:
                    logger.warning(f"Invalid time difference: {dt}. Resetting.")
                    dt = self.pid_sample_time
                
                # Get current and target states
                with self.shared_state.lock:
                    current_roll = self.shared_state.current_roll
                    current_pitch = self.shared_state.current_pitch
                    current_yaw = self.shared_state.current_yaw
                    
                    target_roll = self.shared_state.target_roll
                    target_pitch = self.shared_state.target_pitch
                    target_yaw = self.shared_state.target_yaw
                
                # Update PID controllers with current time difference
                self.pid_roll.sample_time = dt
                self.pid_pitch.sample_time = dt
                self.pid_yaw.sample_time = dt
                
                # Update PID setpoints
                self.pid_roll.setpoint = target_roll
                self.pid_pitch.setpoint = target_pitch
                
                # Calculate yaw error with proper angle normalization
                yaw_error = self.normalize_angle_error(current_yaw, target_yaw)
                self.pid_yaw.setpoint = current_yaw + yaw_error  # Temporary setpoint for PID
                
                # Compute PID corrections
                try:
                    roll_correction = self.pid_roll(current_roll)
                    pitch_correction = self.pid_pitch(current_pitch)
                    yaw_correction = self.pid_yaw(current_yaw)
                except Exception as pid_error:
                    logger.error(f"PID Computation Error: {pid_error}")
                    # Fallback to zero corrections
                    roll_correction = 0
                    pitch_correction = 0
                    yaw_correction = 0
                
                # Compute motor RPMs based on PID corrections
                try:
                    # Scale down the corrections for more reasonable values
                    roll_correction_scaled = roll_correction * 0.2
                    pitch_correction_scaled = pitch_correction * 0.2
                    yaw_correction_scaled = yaw_correction * 0.1
                    
                    # Calculate linear velocity and heading
                    vel_ref = math.sqrt(roll_correction_scaled**2 + pitch_correction_scaled**2)
                    theta_ref = math.atan2(pitch_correction_scaled, roll_correction_scaled)
                    omega_z_ref = yaw_correction_scaled
                    
                    # Compute motor RPMs
                    motor_rpms = self.compute_motor_rpms(vel_ref, theta_ref, omega_z_ref)
                    
                    # Drive motors
                    self.drive_motors(motor_rpms)
                except Exception as motor_error:
                    logger.error(f"Motor Control Error: {motor_error}")
                    self.stop_motors()
                
                # Log current state for debugging
                logger.info(f"Current: Roll={current_roll:.2f}, Pitch={current_pitch:.2f}, Yaw={current_yaw:.2f}")
                logger.info(f"Target:  Roll={target_roll:.2f}, Pitch={target_pitch:.2f}, Yaw={target_yaw:.2f}")
                logger.info(f"Yaw Error: {yaw_error:.2f}")
                logger.info(f"Corrections: Roll={roll_correction:.2f}, Pitch={pitch_correction:.2f}, Yaw={yaw_correction:.2f}")
                
                # Sleep to control update rate
                time.sleep(self.pid_sample_time)
        except Exception as e:
            logger.error(f"Motor Control Thread Error: {e}")
        finally:
            self.stop_motors()
            logger.info("Motor Control Thread Terminated")
    
    def get_shared_state(self):
        return self.shared_state

    def run(self):
        """Start all threads and manage the robot control system."""
        try:
            # Create and start threads
            imu_thread = threading.Thread(target=self.imu_thread_func, daemon=True)
            http_thread = threading.Thread(target=self.http_server_thread_func, daemon=True)
            motor_thread = threading.Thread(target=self.motor_control_thread_func, daemon=True)
            
            imu_thread.start()
            #http_thread.start()
            motor_thread.start()
            
            # Wait for threads to complete (which they won't unless an exception occurs)
            imu_thread.join()
            #http_thread.join()
            motor_thread.join()
        
        except KeyboardInterrupt:
            logger.info("Stopping robot control system...")
        except Exception as e:
            logger.error(f"Critical error in robot control system: {e}")
        finally:
            # Ensure motors are stopped
            self.stop_motors()
            # Close GPIO
            lgpio.gpiochip_close(self.h)
            logger.info("Robot control system shutdown complete.")

def main():
    # Create and run the atlasstone robot controller
    controller = AtlasstoneController()
    controller.run()

if __name__ == "__main__":
    main()
