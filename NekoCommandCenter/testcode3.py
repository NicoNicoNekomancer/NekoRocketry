import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import serial
import serial.tools.list_ports
from collections import deque
import csv
from datetime import datetime
import os

class MagnetometerCalibrator:
    def __init__(self):
        self.serialport = None
        self.device_name = None
        
        # Reduce history size for better performance
        self.mag_x = deque(maxlen=1000)
        self.mag_y = deque(maxlen=1000)
        self.mag_z = deque(maxlen=1000)
        
        # Calibration parameters
        self.bias = np.array([0.0, 0.0, 0.0])
        self.transform_matrix = np.eye(3)
        
        # Create a 'calibration_data' directory in the script's location
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.output_dir = os.path.join(self.script_dir, 'calibration_data')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create output files with timestamp and full path
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.raw_data_file = os.path.join(self.output_dir, f'magnetometer_data_{self.timestamp}.csv')
        self.calibration_file = os.path.join(self.output_dir, f'calibration_parameters_{self.timestamp}.txt')
        
        # Setup the plot
        self.fig = plt.figure(figsize=(15, 5))
        self.ax_raw = self.fig.add_subplot(131, projection='3d')
        self.ax_hard = self.fig.add_subplot(132, projection='3d')
        self.ax_soft = self.fig.add_subplot(133, projection='3d')
        
        # Add text display for calibration parameters
        self.text_display = self.fig.add_axes([0.1, 0.01, 0.8, 0.05])
        self.text_display.axis('off')
        
        for ax in [self.ax_raw, self.ax_hard, self.ax_soft]:
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            
        # Create and write header to raw data file
        try:
            with open(self.raw_data_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'z'])
            print(f"\nCreated raw data file at:\n{self.raw_data_file}")
        except Exception as e:
            print(f"Error creating raw data file: {e}")
            raise
            
        # Try to connect to device on startup
        self.connect_to_device()
        
        # Counter for updating calibration
        self.update_counter = 0
        self.update_interval = 10

    def connect_to_device(self):
        try:
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if port.pid == 0x8cb0 and port.vid in {0x5b99, 0x732B}:
                    self.serialport = serial.Serial(port.device, 115200, timeout=0.1)
                    if port.vid == 0x5b99:
                        self.device_name = "Nekonav"
                    elif port.vid == 0x732B:
                        self.device_name = "Nekonet Receiver"
                    print(f"Connected to {self.device_name} on {port.device}")
                    time.sleep(1)
                    self.serialport.readline()
                    return True
            print("No matching devices found.")
            return False
        except serial.SerialException as e:
            print(f"Could not connect to device: {e}")
            self.serialport = None
            return False

    def save_raw_data(self, data):
        try:
            with open(self.raw_data_file, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # Only write header if the file is empty
                if f.tell() == 0:
                    writer.writerow(['x', 'y', 'z'])
                writer.writerow(data)
        except Exception as e:
            print(f"Error saving raw data: {e}")

    def save_calibration_parameters(self):
        try:
            with open(self.calibration_file, 'w', encoding='utf-8') as f:
                f.write("Hard Iron Calibration (Bias):\n")
                f.write(f"X: {self.bias[0]:.6f}\n")
                f.write(f"Y: {self.bias[1]:.6f}\n")
                f.write(f"Z: {self.bias[2]:.6f}\n\n")
                
                f.write("Soft Iron Calibration (Transform Matrix):\n")
                for row in self.transform_matrix:
                    f.write(f"{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}\n")
                
                f.write("\nMagneto1.2 Format:\n")
                f.write("Hard iron (bias) x y z [uT]:\n")
                f.write(f"{self.bias[0]:.6f} {self.bias[1]:.6f} {self.bias[2]:.6f}\n\n")
                f.write("Soft iron (transformation matrix):\n")
                for row in self.transform_matrix:
                    f.write(f"{row[0]:.6f} {row[1]:.6f} {row[2]:.6f}\n")
            print(f"\nUpdated calibration parameters file at:\n{self.calibration_file}")
        except Exception as e:
            print(f"Error saving calibration parameters: {e}")

    def get_imu_data(self):
        if not self.serialport:
            if not self.connect_to_device():
                return None

        try:
            line = str(self.serialport.readline(), 'utf-8')
            if not line or not line.startswith("LOG: "):
                return None
                
            data_str = line[5:].strip()
            vals = data_str.split(',')
            
            if len(vals) != 3:
                return None
                
            return np.array([float(val.strip()) for val in vals])
            
        except (ValueError, serial.SerialException) as e:
            print(f"Error reading from device: {e}")
            self.serialport = None
            return None

    def calculate_calibration_parameters(self):
        if len(self.mag_x) < 10:
            return
            
        data = np.array(list(zip(self.mag_x, self.mag_y, self.mag_z)))
        
        # Hard Iron Calibration
        self.bias = np.mean(data, axis=0)
        
        # Soft Iron Calibration
        centered_data = data - self.bias
        covariance = np.cov(centered_data.T)
        eigenvals, eigenvecs = np.linalg.eig(covariance)
        
        # Create scaling matrix
        max_eigenval = np.max(eigenvals)
        scaling = np.sqrt(max_eigenval / eigenvals)
        scaling_matrix = np.diag(scaling)
        
        # Calculate transform matrix
        self.transform_matrix = np.dot(eigenvecs, np.dot(scaling_matrix, eigenvecs.T))
        
        # Save calibration parameters
        self.save_calibration_parameters()

    def animate(self, frame):
        # Collect data
        for _ in range(10):
            raw_data = self.get_imu_data()
            if raw_data is None:
                continue
                
            # Save raw data to file
            self.save_raw_data(raw_data)
            
            self.mag_x.append(raw_data[0])
            self.mag_y.append(raw_data[1])
            self.mag_z.append(raw_data[2])

        if len(self.mag_x) < 1:
            return

        self.update_counter += 1
        if self.update_counter >= self.update_interval:
            self.calculate_calibration_parameters()
            self.update_counter = 0

        for ax in [self.ax_raw, self.ax_hard, self.ax_soft]:
            ax.cla()
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

        self.ax_raw.scatter(list(self.mag_x), list(self.mag_y), list(self.mag_z), 
                          c='r', alpha=0.5, s=10)
        self.ax_raw.set_title('Raw Data')

        if len(self.mag_x) >= 10:
            raw_data = np.array(list(zip(self.mag_x, self.mag_y, self.mag_z))).T
            hard_iron_data, soft_iron_data = self.apply_calibration(raw_data)
            
            self.ax_hard.scatter(hard_iron_data[0], hard_iron_data[1], hard_iron_data[2], 
                               c='g', alpha=0.5, s=10)
            self.ax_hard.set_title('Hard Iron Corrected')
            
            self.ax_soft.scatter(soft_iron_data[0], soft_iron_data[1], soft_iron_data[2], 
                               c='b', alpha=0.5, s=10)
            self.ax_soft.set_title('Soft Iron Corrected')

            cal_text = f'Bias: [{self.bias[0]:.1f}, {self.bias[1]:.1f}, {self.bias[2]:.1f}]\n'
            self.text_display.clear()
            self.text_display.text(0.5, 0.5, cal_text, 
                                 horizontalalignment='center',
                                 verticalalignment='center')

        plt.tight_layout()

    def apply_calibration(self, raw_data):
        hard_iron_corrected = raw_data - self.bias[:, np.newaxis]
        soft_iron_corrected = np.dot(self.transform_matrix, hard_iron_corrected)
        return hard_iron_corrected, soft_iron_corrected

    def run(self):
        if not self.serialport:
            print("No device connected. Please connect a Nekonav device.")
            return

        print(f"\nData will be saved to directory:\n{self.output_dir}")
        print(f"\nFiles being used:")
        print(f"Raw data: {os.path.basename(self.raw_data_file)}")
        print(f"Calibration parameters: {os.path.basename(self.calibration_file)}")
        
        anim = animation.FuncAnimation(self.fig, self.animate, 
                                     interval=50,
                                     cache_frame_data=False)
        plt.show()

if __name__ == "__main__":
    calibrator = MagnetometerCalibrator()
    calibrator.run()