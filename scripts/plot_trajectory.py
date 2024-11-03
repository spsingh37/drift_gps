#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Function to read the text file and extract positions
def read_positions_from_file(file_path):
    propagated_positions = []
    measured_positions = []
    estimated_positions = []
    
    with open(file_path, 'r') as f:
        for line in f:
            # Extract positions for Propagated, Measured, and Estimated
            propagated_str = line.split("Propagated: ")[1].split(" Measured: ")[0]
            measured_str = line.split("Measured: ")[1].split(" Estimated: ")[0]
            estimated_str = line.split("Estimated: ")[1].strip()

            # Convert strings to numpy arrays of floats
            propagated = np.array([float(val) for val in propagated_str.split(",")])
            measured = np.array([float(val) for val in measured_str.split(",")])
            estimated = np.array([float(val) for val in estimated_str.split(",")])

            propagated_positions.append(propagated)
            measured_positions.append(measured)
            estimated_positions.append(estimated)

    # Convert to numpy arrays for easier manipulation
    propagated_positions = np.array(propagated_positions)
    measured_positions = np.array(measured_positions)
    estimated_positions = np.array(estimated_positions)
    
    return propagated_positions, measured_positions, estimated_positions

# Clip function to limit values between -80 and 80
def clip_values(data):
    return np.clip(data, -80, 80)

# Plot each graph separately
def plot_imu_data(file_path):
    # Read positions from file
    propagated, measured, estimated = read_positions_from_file(file_path)

    # 1st Plot: 3D Trajectory plot
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection='3d')
    ax1.plot(clip_values(propagated[:, 0]), clip_values(propagated[:, 1]), clip_values(propagated[:, 2]), label='Propagated', color='b')
    ax1.plot(clip_values(measured[:, 0]), clip_values(measured[:, 1]), clip_values(measured[:, 2]), label='Measured', color='r')
    ax1.plot(clip_values(estimated[:, 0]), clip_values(estimated[:, 1]), clip_values(estimated[:, 2]), label='Estimated', color='g')
    ax1.set_title('3D Trajectory')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_zlabel('Z Position')
    ax1.legend()
    plt.show()

    # 2nd Plot: X Position vs Sequence
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(clip_values(propagated[:, 0]), label='Propagated', color='b')
    ax2.plot(clip_values(measured[:, 0]), label='Measured', color='r')
    ax2.plot(clip_values(estimated[:, 0]), label='Estimated', color='g')
    ax2.set_title('X Position vs Sequence')
    ax2.set_xlabel('Data Point')
    ax2.set_ylabel('X Position')
    ax2.legend()
    plt.show()

    # 3rd Plot: Y Position vs Sequence
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.plot(clip_values(propagated[:, 1]), label='Propagated', color='b')
    ax3.plot(clip_values(measured[:, 1]), label='Measured', color='r')
    ax3.plot(clip_values(estimated[:, 1]), label='Estimated', color='g')
    ax3.set_title('Y Position vs Sequence')
    ax3.set_xlabel('Data Point')
    ax3.set_ylabel('Y Position')
    ax3.legend()
    plt.show()

    # 4th Plot: Z Position vs Sequence
    fig4 = plt.figure()
    ax4 = fig4.add_subplot(111)
    ax4.plot(clip_values(propagated[:, 2]), label='Propagated', color='b')
    ax4.plot(clip_values(measured[:, 2]), label='Measured', color='r')
    ax4.plot(clip_values(estimated[:, 2]), label='Estimated', color='g')
    ax4.set_title('Z Position vs Sequence')
    ax4.set_xlabel('Data Point')
    ax4.set_ylabel('Z Position')
    ax4.legend()
    plt.show()

# Call the plot function with the path to your .txt file
file_path = '/home/neofelis/VRX/drift/log/vanilla_est_pose_log.txt'
plot_imu_data(file_path)
