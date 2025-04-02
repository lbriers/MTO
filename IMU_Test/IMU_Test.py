import board
import busio
import digitalio
import adafruit_lsm9ds1

# Initialize SPI bus
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)

# Define chip select pins
csag = digitalio.DigitalInOut(board.D8)  # CE0 for Accel/Gyro
csm = digitalio.DigitalInOut(board.D7)   # CE1 for Magnetometer

# Create LSM9DS1 sensor object
sensor = adafruit_lsm9ds1.LSM9DS1_SPI(spi, csag, csm)

print("Reading LSM9DS1 Sensor Data...")
while True:
    accel_x, accel_y, accel_z = sensor.acceleration
    gyro_x, gyro_y, gyro_z = sensor.gyro
    mag_x, mag_y, mag_z = sensor.magnetic
    temp = sensor.temperature

    print(f"Accel: {accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f} m/s²")
    print(f"Gyro: {gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f} dps")
    print(f"Mag: {mag_x:.2f}, {mag_y:.2f}, {mag_z:.2f} uT")
    print(f"Temp: {temp:.2f} °C\n")
