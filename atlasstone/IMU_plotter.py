# -*- coding: utf-8 -*-
import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a pandas DataFrame
csv_filename = "sensor_data.csv"
df = pd.read_csv(csv_filename)

# Plot Acceleration data (Accel_X, Accel_Y, Accel_Z)
plt.figure(figsize=(10, 6))

plt.subplot(3, 1, 1)
plt.plot(df['Index'], df['Accel_X (m/s^2)'], label='Accel_X (m/s^2)', color='r')
plt.plot(df['Index'], df['Accel_Y (m/s^2)'], label='Accel_Y (m/s^2)', color='g')
plt.plot(df['Index'], df['Accel_Z (m/s^2)'], label='Accel_Z (m/s^2)', color='b')
plt.xlabel('Index')
plt.ylabel('Acceleration (m/s^2)')
plt.title('Acceleration vs Index')
plt.legend()

# Plot Magnetometer data (Mag_X, Mag_Y, Mag_Z)
plt.subplot(3, 1, 2)
plt.plot(df['Index'], df['Mag_X (gauss)'], label='Mag_X (gauss)', color='r')
plt.plot(df['Index'], df['Mag_Y (gauss)'], label='Mag_Y (gauss)', color='g')
plt.plot(df['Index'], df['Mag_Z (gauss)'], label='Mag_Z (gauss)', color='b')
plt.xlabel('Index')
plt.ylabel('Magnetometer (gauss)')
plt.title('Magnetometer vs Index')
plt.legend()

# Plot Gyroscope data (Gyro_X, Gyro_Y, Gyro_Z)
plt.subplot(3, 1, 3)
plt.plot(df['Index'], df['Gyro_X (rad/s)'], label='Gyro_X (rad/s)', color='r')
plt.plot(df['Index'], df['Gyro_Y (rad/s)'], label='Gyro_Y (rad/s)', color='g')
plt.plot(df['Index'], df['Gyro_Z (rad/s)'], label='Gyro_Z (rad/s)', color='b')
plt.xlabel('Index')
plt.ylabel('Gyroscope (rad/s)')
plt.title('Gyroscope vs Index')
plt.legend()

# Display the plots
plt.tight_layout()
plt.show()
