import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
filename = 'data/imu_data_20250511_135848.csv'  # replace with your actual filename
df = pd.read_csv(filename)

# Convert timestamp to float
df['timestamp'] = df['timestamp'].astype(float)

# Define column groups
groups = {
    "Accelerometer (Raw)": ['accel_x_raw', 'accel_y_raw', 'accel_z_raw'],
    "Accelerometer (Filtered)": ['accel_x_filtered', 'accel_y_filtered', 'accel_z_filtered'],
    "Gyroscope (Raw)": ['gyro_x_raw', 'gyro_y_raw', 'gyro_z_raw'],
    "Gyroscope (Filtered)": ['gyro_x_filtered', 'gyro_y_filtered', 'gyro_z_filtered'],
    "Magnetometer (Raw)": ['mag_x_raw', 'mag_y_raw', 'mag_z_raw'],
    "Magnetometer (Filtered)": ['mag_x_filtered', 'mag_y_filtered', 'mag_z_filtered'],
    "Magnetometer (Normalized)": ['mag_x_norm', 'mag_y_norm', 'mag_z_norm'],
    "Euler Angles (Raw)": ['roll_raw', 'pitch_raw', 'yaw_raw'],
    "Euler Angles (Filtered)": ['roll_filtered', 'pitch_filtered', 'yaw_filtered'],
    "Euler Angles (Unfiltered)": ['roll_unfiltered', 'pitch_unfiltered', 'yaw_unfiltered'],
    "Quaternion (Filtered)": ['quat_w', 'quat_x', 'quat_y', 'quat_z'],
    "Quaternion (Raw)": ['quat_raw_w', 'quat_raw_x', 'quat_raw_y', 'quat_raw_z'],
    "Magnetometer Magnitude": ['mag_magnitude']
}

# Plot each group individually
for title, cols in groups.items():
    plt.figure(figsize=(10, 4))
    for col in cols:
        plt.plot(df['timestamp'], df[col], label=col)
    plt.title(title)
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.legend(loc='upper right')
    plt.tight_layout()

# Overview with subplots (excluding Quaternion, Magnetometer Magnitude, and Euler Angles (Raw))
excluded = {"Quaternion (Filtered)", "Quaternion (Raw)", "Magnetometer Magnitude", "Euler Angles (Raw)"}
overview_groups = {k: v for k, v in groups.items() if k not in excluded}

n_groups = len(overview_groups)
fig, axes = plt.subplots(n_groups, 1, figsize=(14, 3 * n_groups), sharex=True)

for ax, (title, cols) in zip(axes, overview_groups.items()):
    for col in cols:
        ax.plot(df['timestamp'], df[col], label=col, linewidth=0.8)
    ax.set_title(title)
    ax.set_ylabel("Value")
    ax.legend(loc='upper right', fontsize="small")

axes[-1].set_xlabel("Time (s)")
plt.tight_layout(rect=[0, 0, 1, 0.97])  # Adjust for suptitle

plt.show()
