import sys
import math
import serial
import struct
import threading
import json
import os
import time
from PySide6.QtCore import QTimer, Qt, QThread, Signal
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QTextEdit, QPushButton, QLabel, QLineEdit, QComboBox, QVBoxLayout, QHBoxLayout
from PySide6.Qt3DCore import QTransform
from PySide6.Qt3DExtras import Qt3DWindow, QPlaneMesh, QPhongMaterial, QEntity
from PySide6.QtGui import QVector3D, QMatrix4x4, QColor

class Cube(QEntity):
    def __init__(self, parentEntity):
        super(Cube, self).__init__()

        # build the cube
        side = 15
        self.cubeEntity = QEntity(parentEntity)

        # init params of the 6 planes
        planeTranslations = [[0, -side / 2, 0], [0, +side / 2, 0],
                             [-side / 2, 0, 0], [+side / 2, 0, 0],
                             [0, 0, -side / 2], [0, 0, +side / 2]]
        planeRotations = [[0, 0, 180], [0, 0, 0], [0, 0, 90],
                          [0, 0, 270], [270, 0, 0], [90, 0, 0]]

        # allocate planes
        self.planeEntities = [None for _ in range(6)]
        self.planeMeshes = [None for _ in range(6)]
        self.planeTransforms = [None for _ in range(6)]
        self.materials = [None for _ in range(6)]

        # set a single color for all faces
        single_color = QColor(10, 120, 182)  # Blue color

        # build the planes
        for i in range(6):
            self.planeMeshes[i] = QPlaneMesh()
            self.planeMeshes[i].setWidth(side)
            self.planeMeshes[i].setHeight(side)

            self.planeTransforms[i] = QTransform()
            self.planeTransforms[i].setRotationX(planeRotations[i][0])
            self.planeTransforms[i].setRotationY(planeRotations[i][1])
            self.planeTransforms[i].setRotationZ(planeRotations[i][2])
            self.planeTransforms[i].setTranslation(QVector3D(planeTranslations[i][0], planeTranslations[i][1], planeTranslations[i][2]))

            self.materials[i] = QPhongMaterial(self.cubeEntity)
            self.materials[i].setAmbient(single_color)  # use the single color here

            self.planeEntities[i] = QEntity(self.cubeEntity)
            self.planeEntities[i].addComponent(self.planeMeshes[i])
            self.planeEntities[i].addComponent(self.planeTransforms[i])
            self.planeEntities[i].addComponent(self.materials[i])

        # initial rotation
        self.cubeTransform = QTransform()
        self.cubeEntity.addComponent(self.cubeTransform)

    def applyQuaternion(self, q):
        # Convert quaternion to rotation matrix
        w, x, y, z = q
        rotation_matrix = QMatrix4x4(
            1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w, 0,
            2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w, 0,
            2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y, 0,
            0, 0, 0, 1
        )
        self.cubeTransform.setMatrix(rotation_matrix)


class Cube3DWindow(Qt3DWindow):
    def __init__(self):
        super(Cube3DWindow, self).__init__()

        # Root entity
        self.rootEntity = QEntity()
        self.setRootEntity(self.rootEntity)

        # Camera
        self.camera().lens().setPerspectiveProjection(45, 16 / 9, 0.1, 1000)
        self.camera().setPosition(QVector3D(0, 0, 40))
        self.camera().setViewCenter(QVector3D(0, 0, 0))

        self.cube = Cube(self.rootEntity)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateCubeRotation)
        self.timer.start(20)

        # Initialize serial communication
        self.serial = serial.Serial('COM14', 115200, timeout=1)  # Change 'COM14' to your port

    def updateCubeRotation(self):
        if self.serial.in_waiting > 0:
            try:
                data = self.serial.readline().decode('utf-8').strip()
                q = [float(num) for num in data.split()]
                if len(q) == 4:
                    self.cube.applyQuaternion(q)
            except Exception as e:
                print(f"Error: {e}")


class SerialThread(QThread):
    received = Signal(str)

    def __init__(self, vid, pids, text_widget):
        super().__init__()
        self.vid = vid
        self.pids = pids
        self.text_widget = text_widget
        self.ser = None

    def run(self):
        self.find_device()
        while True:
            if self.ser is not None and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    self.received.emit(f"Received line: {line}\n")
                except serial.SerialException:
                    self.text_widget.append("Device disconnected.\n")
                    self.ser = None

    def find_device(self):
        while True:
            ports = list(serial.tools.list_ports.comports())
            for p in ports:
                if p.vid == self.vid and p.pid in self.pids and "USB Serial Device" in p.description:
                    if p.pid == 0x8CB0:
                        device_name = "Nekonet Receiver"
                    elif p.pid == 0x5B99:
                        device_name = "NekoNav"
                    if self.ser is None or self.ser.port != p.device:
                        if self.ser is not None:
                            self.ser.close()  # Close the previous serial connection
                        self.ser = serial.Serial(p.device, 115200)  # Adjust baud rate as needed
                        self.text_widget.append(f"Connected to {device_name} on port {p.device}\n")
            time.sleep(1)  # Wait for a second before checking again

    def send_data(self, entries, pressureTransducer, altitudeUnit):
        if self.ser is None:
            self.text_widget.append("No device connected. Cannot send data.\n")
            return
        self.text_widget.append("Button1 pressed. Sending data...\n")
        self.ser.write(b'153\n')  # Send 153 to enter READ state
        self.text_widget.append("Data sent.\nWaiting for the handshake message...\n")
        # Send the entry text as bytes
        data = {}
        for name, entry in entries.items():
            value = entry.text()
            if not value:  # If the cell is blank, set the value to 0
                value = 0
            elif name == 'Callsign':
                value = value.ljust(6)  # Pad with spaces to 6 characters
                value = value.encode('utf-8')  # Encode the string to bytes
            elif name == 'Frequency Mhz':
                value = float(value)
                value = struct.pack('f', value)  # Convert the float to bytes
            elif name in ['Drogue Delay [s]', 'Main Delay [s]', 'Backup State']:
                value = int(value)
            elif name == 'Main Altitude':
                value = int(value)
                if altitudeUnit.currentText() == 'Feet':
                    value = round(value * 0.3048)  # Convert feet to meters and round it
            data[name] = value
        # Get the value from the pressureTransducer combo box
        pressureTransducer_value = pressureTransducer.currentText()
        data['Pressure Transducer'] = pressureTransducer_value == 'Yes'
        # Get the value from the altitudeUnit combo box
        data['Altitude Unit'] = altitudeUnit.currentText()
        # Send the data
        for name, value in data.items():
            if isinstance(value, int):
                self.ser.write(struct.pack('i', value))
            elif isinstance(value, bytes):
                self.ser.write(value)
        with open('config.json', 'w') as f:
            json.dump({name: entry.text() for name, entry in entries.items()}, f)


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.setWindowTitle("Application")
        self.resize(800, 600)

        self.central_widget = QWidget()
        self.central_layout = QGridLayout(self.central_widget)

        self.window3D = Cube3DWindow()
        self.widget3D = self.window3D.createWindowContainer(self.window3D)

        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)

        self.entries = {}
        entry_names = ['Callsign', 'Frequency Mhz', 'Drogue Delay [s]', 'Main Delay [s]', 'Backup State']
        for name in entry_names:
            label = QLabel(name)
            line_edit = QLineEdit()
            self.entries[name] = line_edit
            self.central_layout.addWidget(label)
            self.central_layout.addWidget(line_edit)

        self.altitude_label = QLabel('Main Altitude')
        self.altitude_entry = QLineEdit()
        self.altitude_unit = QComboBox()
        self.altitude_unit.addItems(['Feet', 'Meters'])
        self.central_layout.addWidget(self.altitude_label)
        self.central_layout.addWidget(self.altitude_entry)
        self.central_layout.addWidget(self.altitude_unit)

        self.pressure_label = QLabel('Pressure Transducer')
        self.pressure_transducer = QComboBox()
        self.pressure_transducer.addItems(['Yes', 'No'])
        self.central_layout.addWidget(self.pressure_label)
        self.central_layout.addWidget(self.pressure_transducer)

        self.program_button = QPushButton('Program')
        self.program_button.clicked.connect(self.program_button_clicked)
        self.central_layout.addWidget(self.program_button)

        self.serial_thread = SerialThread(0x732B, [0x8CB0, 0x5B99], self.text_edit)
        self.serial_thread.received.connect(self.text_edit.append)
        self.serial_thread.start()

        self.central_layout.addWidget(self.widget3D, 0, 0, 1, 2)
        self.central_layout.addWidget(self.text_edit, 1, 0, 1, 2)
        self.setCentralWidget(self.central_widget)

        self.load_config()

    def load_config(self):
        if os.path.exists('config.json'):
            with open('config.json', 'r') as f:
                content = f.read().strip()
                if content:  # Check if the file is not empty
                    last_config = json.loads(content)
                    for name, entry in self.entries.items():
                        if name in last_config:
                            entry.setText(last_config[name])
                    if 'Pressure Transducer' in last_config:
                        self.pressure_transducer.setCurrentText(last_config['Pressure Transducer'])
                    if 'Altitude Unit' in last_config:
                        self.altitude_unit.setCurrentText(last_config['Altitude Unit'])

    def program_button_clicked(self):
        self.serial_thread.send_data(self.entries, self.pressure_transducer, self.altitude_unit)


if __name__ == '__main__':
    app = QApplication(sys.argv)

    mainWin = MainWindow()
    mainWin.show()

    sys.exit(app.exec())
