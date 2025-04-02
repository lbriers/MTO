import time
import board
import adafruit_lsm9ds1
import csv

# Initialize I2C connection and sensor
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Define the CSV file name
csv_filename = "sensor_data.csv"

# Open the CSV file in write mode and set up the CSV writer
with open(csv_filename, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    
    # Write the header row
    csv_writer.writerow([ 
        "Index",
        "Accel_X (m/s^2)", "Accel_Y (m/s^2)", "Accel_Z (m/s^2)",
        "Mag_X (gauss)", "Mag_Y (gauss)", "Mag_Z (gauss)",
        "Gyro_X (rad/s)", "Gyro_Y (rad/s)", "Gyro_Z (rad/s)",
        "Temperature (C)"
    ])
    
    # Initialize index counter
    index = 1
    
    # Main loop to read sensor data and write to CSV
    while True:
        # Read sensor data
        accel_x, accel_y, accel_z = sensor.acceleration
        mag_x, mag_y, mag_z = sensor.magnetic
        gyro_x, gyro_y, gyro_z = sensor.gyro
        temp = sensor.temperature
        
        # Write the data row to the CSV file with the index as the first column
        csv_writer.writerow([ 
            index,  # Use index instead of timestamp
            accel_x, accel_y, accel_z,
            mag_x, mag_y, mag_z,
            gyro_x, gyro_y, gyro_z,
            temp
        ])
        
        # Print the data to the console (optional)
        print(f"Index: {index} - Accel: ({accel_x:.3f}, {accel_y:.3f}, {accel_z:.3f}) m/s^2 | "
              f"Mag: ({mag_x:.3f}, {mag_y:.3f}, {mag_z:.3f}) gauss | "
              f"Gyro: ({gyro_x:.3f}, {gyro_y:.3f}, {gyro_z:.3f}) rad/s | "
              f"Temp: {temp:.3f} C")
        
        # Increment the index for the next row
        index += 1
        
        # Delay for a second
        time.sleep(0.1)
