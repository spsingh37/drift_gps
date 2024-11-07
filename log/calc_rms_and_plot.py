import numpy as np
import matplotlib.pyplot as plt

def calculate_errors_and_plot(file_path):
    propagated_states = []
    measured_states = []
    estimated_states = []

    # Reading the file
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split(" ")
            # Extracting values from the line and converting them to float
            propagated = tuple(map(float, parts[1][:-1].split(",")))
            measured = tuple(map(float, parts[3][:-1].split(",")))
            estimated = tuple(map(float, parts[5].split(",")))

            propagated_states.append(propagated)
            measured_states.append(measured)
            estimated_states.append(estimated)

    # Converting lists to numpy arrays for easier calculation
    measured_states = np.array(measured_states)
    estimated_states = np.array(estimated_states)

    # Calculating the RMS error
    rms_error = np.sqrt(np.mean((measured_states - estimated_states) ** 2, axis=0))

    # Calculating the maximum absolute error for each component
    max_abs_error = np.max(np.abs(measured_states - estimated_states), axis=0)
    
    print("RMS Error between measured and estimated states (x, y, z):", rms_error)
    print("Max Absolute Error between measured and estimated states (x, y, z):", max_abs_error)

    # Plotting measured vs estimated states for x and y components
    plt.figure(figsize=(15, 5))
    
    # Time-series plot for x component
    plt.subplot(1, 3, 1)
    plt.plot(measured_states[:, 0], label='GPS x', color='blue')
    plt.plot(estimated_states[:, 0], label='Estimated x', color='orange', linestyle='--')
    plt.xlabel('Time Step')
    plt.ylabel('x Value')
    plt.title('GPS vs Estimated (x)')
    plt.legend()

    # Time-series plot for y component
    plt.subplot(1, 3, 2)
    plt.plot(measured_states[:, 1], label='GPS y', color='blue')
    plt.plot(estimated_states[:, 1], label='Estimated y', color='orange', linestyle='--')
    plt.xlabel('Time Step')
    plt.ylabel('y Value')
    plt.title('GPS vs Estimated (y)')
    plt.legend()

    # x-y trajectory plot
    plt.subplot(1, 3, 3)
    plt.plot(measured_states[:, 0], measured_states[:, 1], label='GPS Trajectory', color='blue')
    plt.plot(estimated_states[:, 0], estimated_states[:, 1], label='Estimated Trajectory', color='orange', linestyle='--')
    plt.xlabel('x Value')
    plt.ylabel('y Value')
    plt.title('GPS vs Estimated Trajectory (x-y)')
    plt.legend()
    
    plt.tight_layout()
    plt.show()

    return rms_error, max_abs_error

# Example usage
rms_error, max_abs_error = calculate_errors_and_plot('vanilla_est_pose_log.txt')

