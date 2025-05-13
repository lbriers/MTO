import time
import math
import board
import adafruit_lsm9ds1

i2c = board.I2C()
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

print("Rotate the sensor in all directions (figure-8 motion) for 30 seconds...")
print("Press Ctrl+C to stop early.")

# Initialize min/max values with first read
mag_x, mag_y, mag_z = sensor.magnetic
min_x, max_x = mag_x, mag_x
min_y, max_y = mag_y, mag_y
min_z, max_z = mag_z, mag_z

start_time = time.time()
duration = 30  # seconds

try:
    while time.time() - start_time < duration:
        mag_x, mag_y, mag_z = sensor.magnetic

        min_x = min(min_x, mag_x)
        max_x = max(max_x, mag_x)

        min_y = min(min_y, mag_y)
        max_y = max(max_y, mag_y)

        min_z = min(min_z, mag_z)
        max_z = max(max_z, mag_z)

        print(f"Magnetometer: x={mag_x:.2f}, y={mag_y:.2f}, z={mag_z:.2f}")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nInterrupted by user.")

# Compute offset and scale
offset_x = (max_x + min_x) / 2
offset_y = (max_y + min_y) / 2
offset_z = (max_z + min_z) / 2

scale_x = (max_x - min_x) / 2
scale_y = (max_y - min_y) / 2
scale_z = (max_z - min_z) / 2

avg_scale = (scale_x + scale_y + scale_z) / 3

scale_x = avg_scale / scale_x
scale_y = avg_scale / scale_y
scale_z = avg_scale / scale_z

print("\n--- Magnetometer Calibration Complete ---")
print(f"Offsets: ({offset_x:.4f}, {offset_y:.4f}, {offset_z:.4f})")
print(f"Scales:  ({scale_x:.4f}, {scale_y:.4f}, {scale_z:.4f})")
