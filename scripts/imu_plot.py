#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np

# Lists to store imu data for plotting
angular_velocity_x = []
angular_velocity_y = []
angular_velocity_z = []

linear_acceleration_x = []
linear_acceleration_y = []
linear_acceleration_z = []

sequence = []

def imu_callback(data):
    # Extract angular velocities
    angular_velocity_x.append(data.angular_velocity.x)
    angular_velocity_y.append(data.angular_velocity.y)
    angular_velocity_z.append(data.angular_velocity.z)

    # Extract linear accelerations
    linear_acceleration_x.append(data.linear_acceleration.x)
    linear_acceleration_y.append(data.linear_acceleration.y)
    linear_acceleration_z.append(data.linear_acceleration.z)

    # Update sequence number (every 10th point)
    if len(sequence) == 0:
        sequence.append(1)
    else:
        sequence.append(sequence[-1] + 1)

def plot_imu_data():
    # Reduce the number of points on the x-axis to every 10th point
    step = 10
    reduced_sequence = np.array(sequence)[::step]/10
    reduced_angular_velocity_x = np.array(angular_velocity_x)[::step]
    reduced_angular_velocity_y = np.array(angular_velocity_y)[::step]
    reduced_angular_velocity_z = np.array(angular_velocity_z)[::step]
    reduced_linear_acceleration_x = np.array(linear_acceleration_x)[::step]
    reduced_linear_acceleration_y = np.array(linear_acceleration_y)[::step]
    reduced_linear_acceleration_z = np.array(linear_acceleration_z)[::step]

    # First plot for angular velocities (x, y, z)
    plt.figure(1)
    plt.subplot(211)
    plt.plot(reduced_sequence, reduced_angular_velocity_x, label='Angular Velocity X')
    plt.plot(reduced_sequence, reduced_angular_velocity_y, label='Angular Velocity Y')
    plt.plot(reduced_sequence, reduced_angular_velocity_z, label='Angular Velocity Z')
    plt.xlabel('Data Point Sequence')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('IMU Angular Velocities')
    plt.legend()

    # Second plot for linear accelerations (x, y, z)
    plt.subplot(212)
    plt.plot(reduced_sequence, reduced_linear_acceleration_x, label='Linear Acceleration X')
    plt.plot(reduced_sequence, reduced_linear_acceleration_y, label='Linear Acceleration Y')
    plt.plot(reduced_sequence, reduced_linear_acceleration_z, label='Linear Acceleration Z')
    plt.xlabel('Data Point Sequence')
    plt.ylabel('Linear Acceleration (m/s^2)')
    plt.title('IMU Linear Accelerations')
    plt.legend()

    # Display the plots
    plt.tight_layout()
    plt.show()

def imu_listener():
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("/wamv/sensors/imu/imu/data", Imu, imu_callback)

    # Keep the node running and allow callback functions to process data
    rospy.spin()

    # Once the node shuts down, plot the collected data
    plot_imu_data()

if __name__ == '__main__':
    try:
        imu_listener()
    except rospy.ROSInterruptException:
        pass
