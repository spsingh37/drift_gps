#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

# Global variables to store imu and odometry data
imu_data = []
odom_data = []
prev_imu_time = None
path = []

# Transformation matrix for IMU to robot frame (modify this as needed)
imu_to_robot_transform = np.eye(4)

# Function to transform IMU data
def transform_imu_data(imu_msg, transform_matrix):
    # Convert quaternion to rotation matrix
    orientation_q = imu_msg.orientation
    quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
    
    # Apply the transformation matrix
    transformed_matrix = np.dot(transform_matrix, rotation_matrix)
    
    # Convert back to quaternion
    transformed_quaternion = tf.transformations.quaternion_from_matrix(transformed_matrix)
    
    return transformed_quaternion

# Function to transform odometry data
def transform_odometry_data(odom_msg, transform_matrix):
    # Transform position
    position = np.array([odom_msg.pose.pose.position.x, 
                         odom_msg.pose.pose.position.y, 
                         odom_msg.pose.pose.position.z, 1.0])
    transformed_position = np.dot(transform_matrix, position)
    
    # Transform orientation
    orientation_q = odom_msg.pose.pose.orientation
    quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
    transformed_matrix = np.dot(transform_matrix, rotation_matrix)
    
    transformed_quaternion = tf.transformations.quaternion_from_matrix(transformed_matrix)
    
    return transformed_position, transformed_quaternion

# Function to integrate IMU data and generate path
def integrate_imu_data(imu_data, initial_pose):
    # Initialize path with the starting pose
    path = [initial_pose]  
    # Initialize velocity
    velocity = np.zeros(2)  # Velocity in x and y
    orientation = initial_pose[2]  # Yaw from the initial pose

    prev_time = imu_data[0].header.stamp.secs + imu_data[0].header.stamp.nsecs * 1e-9  # First timestamp

    for i in range(1, len(imu_data)):
        # Calculate current time
        curr_time = imu_data[i].header.stamp.secs + imu_data[i].header.stamp.nsecs * 1e-9
        dt = curr_time - prev_time  # Time difference
        prev_time = curr_time  # Update previous time

        # Get IMU readings
        acc_x = imu_data[i].linear_acceleration.x
        acc_y = imu_data[i].linear_acceleration.y
        angular_velocity_z = imu_data[i].angular_velocity.z

        # Update orientation by integrating angular velocity
        orientation += angular_velocity_z * dt

        # Create a rotation matrix from orientation
        R_matrix = np.array([[np.cos(orientation), -np.sin(orientation)],
                             [np.sin(orientation), np.cos(orientation)]])
        
        # Combine accelerations into a vector
        accel = np.array([acc_x, acc_y])
        
        # Rotate acceleration from IMU frame to world frame
        accel_world = R_matrix @ accel
        
        # Update velocity by integrating acceleration
        velocity += accel_world * dt
        
        # Update position based on velocity
        # New position based on previous position and velocity
        new_pose = np.copy(path[-1])
        new_pose[0] += velocity[0] * dt  # Update x
        new_pose[1] += velocity[1] * dt  # Update y
        new_pose[2] = orientation  # Update orientation

        # Append the new pose to the path
        path.append(new_pose)

        # For debugging, print the current path
        print("Current Path:", path)

    return np.array(path)

# Function to generate a path from odometry data
def generate_odometry_path(odom_data):
    path = []
    for odom_msg in odom_data:
        pose = np.array([odom_msg.pose.pose.position.x,
                         odom_msg.pose.pose.position.y,
                         tf.transformations.euler_from_quaternion([odom_msg.pose.pose.orientation.x,
                                                                   odom_msg.pose.pose.orientation.y,
                                                                   odom_msg.pose.pose.orientation.z,
                                                                   odom_msg.pose.pose.orientation.w])[2]])
        path.append(pose)
    return np.array(path)

# Function to publish a path to RViz
def publish_path(path, publisher, frame_id="robot_frame"):
    path_msg = Path()
    path_msg.header.frame_id = frame_id

    for pose in path:
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        
        path_msg.poses.append(pose_msg)
    
    publisher.publish(path_msg)

# Callback for IMU data
def imu_callback(msg):
    global imu_data
    imu_data.append(msg)

# Callback for Odometry data
def odom_callback(msg):
    global odom_data
    odom_data.append(msg)

# Main function
def main():
    rospy.init_node('imu_odometry_path_visualizer')
    
    # Publishers for IMU-based and odometry-based paths
    imu_path_pub = rospy.Publisher('/imu_path', Path, queue_size=10)
    odom_path_pub = rospy.Publisher('/odom_path', Path, queue_size=10)
    
    # Subscribers to IMU and odometry topics
    rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, imu_callback)
    rospy.Subscriber('/wamv/sensors/position/ground_truth_odometry', Odometry, odom_callback)

    # Initial pose [x, y, theta]
    initial_pose = [0, 0, 0]
    
    rate = rospy.Rate(10)  # Publish at 10 Hz
    
    while not rospy.is_shutdown():
        if len(imu_data) > 1:
            imu_path = integrate_imu_data(imu_data, initial_pose)
            publish_path(imu_path, imu_path_pub, frame_id="robot_frame")
        
        if len(odom_data) > 0:
            odom_path = generate_odometry_path(odom_data)
            publish_path(odom_path, odom_path_pub, frame_id="robot_frame")
        
        rate.sleep()

if __name__ == '__main__':
    main()