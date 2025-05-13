import time
import board
import adafruit_lsm9ds1
import csv

# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Open CSV file in append mode (create file if it doesn't exist)
with open("magnetometer_data.csv", mode="a", newline="") as file:
    writer = csv.writer(file)

    # Write header if the file is empty
    file.seek(0, 2)  # Seek to end of file
    if file.tell() == 0:
        writer.writerow(["Timestamp", "Mag_X (Gauss)", "Mag_Y (Gauss)", "Mag_Z (Gauss)"])

    # Main loop will read the magnetometer values every second and save them to the CSV file
    while True:
        # Read magnetometer values
        mag_x, mag_y, mag_z = sensor.magnetic
        
        # Get current timestamp
        timestamp = time.time()

        # Print values
        print(
            "Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})".format(mag_x, mag_y, mag_z)
        )

        # Save values to CSV
        writer.writerow([timestamp, mag_x, mag_y, mag_z])

        # Delay for a short period
        time.sleep(0.001)  # 0.001 second delay between readings
