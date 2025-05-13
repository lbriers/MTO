import time
import board
import adafruit_lsm9ds1
import matplotlib.pyplot as plt
from collections import deque

# Initialize I2C and sensor
i2c = board.I2C()
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Set up real-time plot
plt.ion()  # Interactive mode
fig, ax = plt.subplots()
xs = deque(maxlen=100)  # Store last 100 readings
ys_x = deque(maxlen=100)
ys_y = deque(maxlen=100)
ys_z = deque(maxlen=100)

line_x, = ax.plot([], [], 'r-', label='Mag X')
line_y, = ax.plot([], [], 'g-', label='Mag Y')
line_z, = ax.plot([], [], 'b-', label='Mag Z')
ax.set_xlabel('Time (samples)')
ax.set_ylabel('Magnetic Field (Gauss)')
ax.legend()
ax.grid(True)

print("Logging magnetometer data. Press Ctrl+C to stop.")

try:
    index = 0
    while True:
        # Read magnetometer data
        mag_x, mag_y, mag_z = sensor.magnetic
        
        # Update plot data
        xs.append(index)
        ys_x.append(mag_x)
        ys_y.append(mag_y)
        ys_z.append(mag_z)
        
        # Update plot
        line_x.set_data(xs, ys_x)
        line_y.set_data(xs, ys_y)
        line_z.set_data(xs, ys_z)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.flush_events()
        
        # Print to console (optional)
        print(f"Mag: X={mag_x:.2f} G, Y={mag_y:.2f} G, Z={mag_z:.2f} G")
        
        index += 1
        time.sleep(0.1)  # Adjust sampling rate

except KeyboardInterrupt:
    print("Stopped logging.")

# Keep plot open after stopping
plt.ioff()
plt.show()
