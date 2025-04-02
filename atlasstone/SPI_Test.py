import time
import csv
import board
import adafruit_lsm9ds1
from digitalio import DigitalInOut, Direction

# SPI connection
spi = board.SPI()
csag = DigitalInOut(board.D5)
csag.direction = Direction.OUTPUT
csag.value = True
csm = DigitalInOut(board.D6)
csm.direction = Direction.OUTPUT
csm.value = True
sensor = adafruit_lsm9ds1.LSM9DS1_SPI(spi, csag, csm)

# CSV file setup
csv_filename = "lsm9ds1_data.csv"
header = [
    "Timestamp", "Accel_X", "Accel_Y", "Accel_Z", 
    "Mag_X", "Mag_Y", "Mag_Z", 
    "Gyro_X", "Gyro_Y", "Gyro_Z", "Temperature"
]

# Create and write header if file doesn't exist
try:
    with open(csv_filename, "x", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)
except FileExistsError:
    pass  # File already exists, skip header writing

# Main loop
while True:
    # Read sensor data
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    temp = sensor.temperature
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    
    # Print values
    print(f"[{timestamp}] Accel: ({accel_x:.3f}, {accel_y:.3f}, {accel_z:.3f}) m/s^2")
    print(f"Mag: ({mag_x:.3f}, {mag_y:.3f}, {mag_z:.3f}) gauss")
    print(f"Gyro: ({gyro_x:.3f}, {gyro_y:.3f}, {gyro_z:.3f}) rad/sec")
    print(f"Temp: {temp:.3f}C\n")
    
    # Save to CSV
    with open(csv_filename, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z, temp])
    
    # Wait for next sample
    time.sleep(0.1)
