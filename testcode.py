import serial
import time
import pygame
from pygame.locals import *
import threading  # Import the threading module

# Global variables
last_time = time.time()
last_angles = {'x': 0, 'y': 0, 'z': 0}  # Initial angles
acceleration = [0, 0, 0]  # Initial acceleration
kalman_data = [0, 0, 0]  # Initial Kalman filtered data

# Initialize Pygame
pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

# Load a custom OBJ model
obj_model = pygame.image.load("your_model.obj")  # Load your custom OBJ model image

# Function to update the model based on Kalman filtered data
def update_model():
    global last_time, acceleration, kalman_data, last_angles

    # Calculate time difference
    current_time = time.time()
    delta_time = current_time - last_time
    last_time = current_time

    # Apply threshold to Kalman filtered data to filter out small fluctuations
    kalman_threshold = 0.01  # Adjust threshold as needed
    if any(abs(d) > kalman_threshold for d in kalman_data):
        # Rotate the model based on Kalman filtered data
        # Implement rotation logic here based on your custom OBJ model
        pass

    # Update acceleration
    acceleration = [round(a, 2) for a in acceleration]  # Round acceleration values

    # Calculate current angles in x, y, and z axes
    current_angles = {'x': degrees(obj_model.axis.x), 'y': degrees(obj_model.axis.y), 'z': degrees(obj_model.axis.z)}

    # Check if there's been a change of more than 1 degree in any axis
    if any(abs(current_angles[axis] - last_angles[axis]) > 1 for axis in ['x', 'y', 'z']):
        print("Acceleration (g):", acceleration)
        print("Current angles (degrees):", current_angles)

        # Update last angles
        last_angles = current_angles

# Function to read sensor data from Arduino
def read_sensor_data(ser):
    global acceleration, kalman_data
    while True:
        try:
            # Read data from serial port
            data = ser.readline().decode().strip()
            if data:  # Check if data is not empty
                # Split the string at each comma and convert values to floats
                values = [float(val) for val in data.split(',')]
                # Extract acceleration and Kalman filtered data
                acceleration = values[:3]
                kalman_data = values[3:]
                # Update the model based on sensor data
                update_model()
        except Exception as e:
            print("Error reading data:", e)

# Main function
def main():
    # Initialize serial communication with Arduino
    ser = serial.Serial('COM3', 115200)  # Adjust COM port and baud rate as needed
    ser.flush()  # Flush input buffer to ensure data integrity
    print("Serial monitor started.")

    # Create a separate thread to read sensor data continuously
    threading.Thread(target=read_sensor_data, args=(ser,), daemon=True).start()

    # Main loop
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        # Clear the screen
        screen.fill((255, 255, 255))

        # Draw the model
        # Implement drawing logic here based on your custom OBJ model
        # Use pygame functions to draw the model on the screen

        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()
