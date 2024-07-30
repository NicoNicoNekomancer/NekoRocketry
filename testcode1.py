import sys
import json
import serial
import serial.tools.list_ports
from PySide6.QtCore import QTimer, Qt, QSize
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QWidget, QLabel,
    QGridLayout, QTextEdit, QSizePolicy, QLineEdit, QSpinBox, QDoubleSpinBox, QDialog,
    QFormLayout, QMenuBar, QMenu
)
from PySide6.QtGui import QFont, QIcon, QColor, QPalette, QPainter, QBrush, QPixmap, QAction
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.QtGui import QVector3D, QMatrix4x4

class Cube(Qt3DCore.QEntity):
    def __init__(self, parentEntity):
        super(Cube, self).__init__()

        # build the cube
        side = 15
        self.cubeEntity = Qt3DCore.QEntity(parentEntity)

        # init params of the 6 planes
        planeTranslations = [[0, -side / 2, 0], [0, +side / 2, 0],
                             [-side / 2, 0, 0], [+side / 2, 0, 0],
                             [0, 0, -side / 2], [0, 0, +side / 2]]
        planeRotations = [[0, 0, 180], [0, 0, 0], [0, 0, 90],
                          [0, 0, 270], [270, 0, 0], [90, 0, 0]]

        # allocate planes
        self.planeEntities = [None for i in range(6)]
        self.planeMeshes = [None for i in range(6)]
        self.planeTransforms = [None for i in range(6)]
        self.materials = [None for i in range(6)]

        # set a single color for all faces
        single_color = QColor(10, 120, 182)  # Blue color

        # build the planes
        for i in range(0, 6):
            self.planeMeshes[i] = Qt3DExtras.QPlaneMesh()
            self.planeMeshes[i].setWidth(side)
            self.planeMeshes[i].setHeight(side)

            self.planeTransforms[i] = Qt3DCore.QTransform()
            self.planeTransforms[i].setRotationX(planeRotations[i][0])
            self.planeTransforms[i].setRotationY(planeRotations[i][1])
            self.planeTransforms[i].setRotationZ(planeRotations[i][2])
            self.planeTransforms[i].setTranslation(QVector3D(planeTranslations[i][0], planeTranslations[i][1], planeTranslations[i][2]))

            self.materials[i] = Qt3DExtras.QPhongMaterial(self.cubeEntity)
            self.materials[i].setAmbient(single_color)  # use the single color here

            self.planeEntities[i] = Qt3DCore.QEntity(self.cubeEntity)
            self.planeEntities[i].addComponent(self.planeMeshes[i])
            self.planeEntities[i].addComponent(self.planeTransforms[i])
            self.planeEntities[i].addComponent(self.materials[i])

        # initial rotation
        self.cubeTransform = Qt3DCore.QTransform()
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


class Cube3DWindow(Qt3DExtras.Qt3DWindow):
    def __init__(self, main_window):
        super(Cube3DWindow, self).__init__()
        self.main_window = main_window

        # Root entity
        self.rootEntity = Qt3DCore.QEntity()
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
        self.serial = None
        self.device_name = "Not connected"

        # Timer to periodically check the connection status
        self.connection_check_timer = QTimer(self)
        self.connection_check_timer.timeout.connect(self.check_connection)
        self.connection_check_timer.start(1000)  # Check every second

    def connectSerial(self):
        try:
            # Search for ports
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if port.pid == 0x8cb0 and port.vid in {0x5b99, 0x732B}:
                    self.serial = serial.Serial(port.device, 115200, timeout=1)
                    if port.vid == 0x5b99:
                        self.device_name = "Nekonav"
                    elif port.vid == 0x732B:
                        self.device_name = "Nekonet Receiver"
                    print(f"Connected to {self.device_name} on {port.device}")
                    return
            print("No matching devices found.")
        except serial.SerialException:
            print("Could not connect to any device.")
            self.serial = None

    def disconnectSerial(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected from device.")
            self.serial = None
            self.device_name = "Not connected"

    def updateCubeRotation(self):
        try:
            if self.serial and self.serial.is_open and self.serial.in_waiting > 0:
                try:
                    data = self.serial.readline().decode('utf-8').strip()
                    q = [float(num) for num in data.split()]
                    if len(q) == 4:
                        self.cube.applyQuaternion(q)
                except Exception as e:
                    print(f"Error: {e}")
                    self.serial = None
                    self.connectSerial()
        except serial.SerialException as e:
            print(f"Serial Exception: {e}")
            self.serial = None
            self.device_name = "Not connected"
            self.main_window.update_serial_ui()

    def sendData(self, data):
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(f"{data}\n".encode('utf-8'))
                print(f"Data sent: {data}")
            except Exception as e:
                print(f"Error sending data: {e}")
                self.serial = None

    def check_connection(self):
        if self.serial and not self.serial.is_open:
            print("Serial port closed unexpectedly.")
            self.disconnectSerial()
            self.device_name = "Not connected"
            self.main_window.update_serial_ui()
            # Emulate a button click to reflect the disconnection
            self.connect_to_device()


class SettingsDialog(QDialog):
    def __init__(self, config, parent=None):
        super(SettingsDialog, self).__init__(parent)
        self.config = config

        self.setWindowTitle("Altimeter Settings")

        self.layout = QFormLayout(self)

        # Callsign (6 character string)
        self.callsign_input = QLineEdit(self.config.get("Callsign", ""))
        self.callsign_input.setMaxLength(6)
        self.layout.addRow("Callsign:", self.callsign_input)

        # Frequency (float with three decimal places)
        self.frequency_input = QDoubleSpinBox()
        self.frequency_input.setDecimals(3)
        self.frequency_input.setRange(0.0, 1000.0)
        self.frequency_input.setValue(float(self.config.get("Frequency Mhz", 0.0)))
        self.layout.addRow("Frequency (MHz):", self.frequency_input)

        # Drogue Delay (short)
        self.drogue_delay_input = QSpinBox()
        self.drogue_delay_input.setRange(0, 32767)
        self.drogue_delay_input.setValue(int(self.config.get("Drogue Delay [s]", 0)))
        self.layout.addRow("Drogue Delay [s]:", self.drogue_delay_input)

        # Main Delay (short)
        self.main_delay_input = QSpinBox()
        self.main_delay_input.setRange(0, 32767)
        self.main_delay_input.setValue(int(self.config.get("Main Delay [s]", 1)))
        self.layout.addRow("Main Delay [s]:", self.main_delay_input)

        # Sample Rate (int)
        self.sample_rate_input = QSpinBox()
        self.sample_rate_input.setRange(0, 2147483647)
        self.sample_rate_input.setValue(int(self.config.get("Sample Rate [ms]", 500)))
        self.layout.addRow("Sample Rate [ms]:", self.sample_rate_input)

        # Save button
        self.save_button = QPushButton("Save")
        self.save_button.clicked.connect(self.save_settings)
        self.layout.addWidget(self.save_button)

    def save_settings(self):
        self.config["Callsign"] = self.callsign_input.text()
        self.config["Frequency Mhz"] = self.frequency_input.value()
        self.config["Drogue Delay [s]"] = self.drogue_delay_input.value()
        self.config["Main Delay [s]"] = self.main_delay_input.value()
        self.config["Sample Rate [ms]"] = self.sample_rate_input.value()
        self.accept()


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.window3D = Cube3DWindow(self)  # Pass 'self' as the main_window argument
        self.widget3D = self.createWindowContainer(self.window3D)

        self.central_widget = QWidget()
        self.grid_layout = QGridLayout(self.central_widget)
        self.setCentralWidget(self.central_widget)

        self.logo_placeholder = QLabel(alignment=Qt.AlignCenter)
        self.logo_placeholder.setPixmap(QPixmap("Logo.png"))
        self.logo_placeholder.setScaledContents(True)
        self.logo_placeholder.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.serial_log = QTextEdit()
        self.serial_log.setReadOnly(True)
        self.serial_log.setPlaceholderText("Serial Communication Log")
        self.serial_log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.widget3D.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.btn_read_flight = QPushButton("Read Flight")
        self.btn_read_flight.clicked.connect(lambda: self.send_data(153))

        self.btn_delete_flight = QPushButton("Delete Flight")
        self.btn_delete_flight.clicked.connect(lambda: self.send_data(52))

        self.btn_program_device = QPushButton("Program Device")
        self.btn_program_device.clicked.connect(lambda: self.send_data(231))

        self.btn_calibrate_altimeter = QPushButton("Calibrate Altimeter")
        self.btn_calibrate_altimeter.clicked.connect(lambda: self.send_data(262))

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.connect_to_device)

        self.device_label = QLabel("Device: Not connected")

        self.top_layout = QHBoxLayout()
        self.top_layout.addWidget(self.btn_connect)
        self.top_layout.addWidget(self.device_label)
        self.top_layout.addStretch()

        self.grid_layout.addLayout(self.top_layout, 0, 0, 1, 5, alignment=Qt.AlignLeft)
        self.grid_layout.addWidget(self.logo_placeholder, 1, 0, 1, 5)
        self.grid_layout.addWidget(self.widget3D, 2, 0, 1, 5)
        self.grid_layout.addWidget(self.serial_log, 2, 0, 1, 5)

        self.buttons_layout = QHBoxLayout()
        self.buttons_layout.addStretch()
        self.buttons_layout.addWidget(self.btn_read_flight)
        self.buttons_layout.addWidget(self.btn_delete_flight)
        self.buttons_layout.addWidget(self.btn_program_device)
        self.buttons_layout.addWidget(self.btn_calibrate_altimeter)
        self.buttons_layout.addStretch()
        self.grid_layout.addLayout(self.buttons_layout, 3, 0, 1, 5)

        self.widget3D.setVisible(False)
        self.serial_log.setVisible(False)
        self.logo_placeholder.setVisible(True)

        # Load configuration
        self.load_config()

        # Create a menu bar
        self.menu_bar = QMenuBar()
        self.setMenuBar(self.menu_bar)

        # Add "Settings" to the menu bar
        self.settings_menu = QMenu("Settings", self)
        self.menu_bar.addMenu(self.settings_menu)

        self.altimeter_settings_action = QAction("Altimeter Settings", self)
        self.altimeter_settings_action.triggered.connect(self.open_settings_dialog)
        self.settings_menu.addAction(self.altimeter_settings_action)

        self.theme_toggle_action = QAction("Toggle Theme", self)
        self.theme_toggle_action.triggered.connect(self.toggle_theme)
        self.settings_menu.addAction(self.theme_toggle_action)

        self.cube_toggle_action = QAction("Toggle Cube Display", self)
        self.cube_toggle_action.triggered.connect(self.toggle_cube_display)
        self.settings_menu.addAction(self.cube_toggle_action)

        self.serial_toggle_action = QAction("Toggle Serial Display", self)
        self.serial_toggle_action.triggered.connect(self.toggle_serial_display)
        self.settings_menu.addAction(self.serial_toggle_action)

        self.resize(800, 600)
        self.setWindowTitle("Application")

        # Apply initial theme
        self.apply_theme(self.config.get("Theme", "Light Mode"))

        self.show()

        # Timer to check the serial connection status
        self.connection_timer = QTimer(self)
        self.connection_timer.timeout.connect(self.check_serial_connection)
        self.connection_timer.start(1000)  # Check every second

    def load_config(self):
        try:
            with open("config.json", "r") as f:
                self.config = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            self.config = {
                "Callsign": "",
                "Frequency Mhz": "0.000",
                "Drogue Delay [s]": "0",
                "Main Delay [s]": "1",
                "Backup State": "1",
                "Main Altitude": "1500",
                "Theme": "Light Mode"
            }

    def open_settings_dialog(self):
        dialog = SettingsDialog(self.config, self)
        if dialog.exec():
            self.load_config()

    def toggle_theme(self):
        current_theme = self.config.get("Theme", "Light Mode")
        new_theme = "Dark Mode" if current_theme == "Light Mode" else "Light Mode"
        self.config["Theme"] = new_theme

        with open("config.json", "w") as f:
            json.dump(self.config, f, indent=4)

        self.apply_theme(new_theme)

    def apply_theme(self, theme):
        if theme == "Dark Mode":
            self.setStyleSheet("background-color: #2b2b2b; color: #ffffff;")
        else:
            self.setStyleSheet("")

    def toggle_cube_display(self):
        self.widget3D.setVisible(not self.widget3D.isVisible())
        self.update_display()

    def toggle_serial_display(self):
        self.serial_log.setVisible(not self.serial_log.isVisible())
        self.update_display()

    def update_display(self):
        self.logo_placeholder.setVisible(not (self.widget3D.isVisible() or self.serial_log.isVisible()))

        if self.widget3D.isVisible() and self.serial_log.isVisible():
            self.grid_layout.addWidget(self.serial_log, 2, 0, 1, 2)
            self.grid_layout.addWidget(self.widget3D, 2, 2, 1, 3)
        elif self.widget3D.isVisible():
            self.grid_layout.addWidget(self.widget3D, 2, 0, 1, 5)
        elif self.serial_log.isVisible():
            self.grid_layout.addWidget(self.serial_log, 2, 0, 1, 5)

    def send_data(self, data):
        self.window3D.sendData(data)

    def connect_to_device(self):
        if self.window3D.serial and self.window3D.serial.is_open:
            self.window3D.disconnectSerial()
            self.device_label.setText("Device: Not connected")
            self.btn_connect.setText("Connect")
        else:
            self.window3D.connectSerial()
            if self.window3D.serial and self.window3D.serial.is_open:
                self.device_label.setText(f"Device: {self.window3D.device_name}")
                self.btn_connect.setText("Disconnect")
            else:
                self.device_label.setText("Device: Not connected")

    def check_serial_connection(self):
        if self.window3D.serial and not self.window3D.serial.is_open:
            self.update_serial_ui()

    def update_serial_ui(self):
        self.device_label.setText("Device: Not connected")
        self.btn_connect.setText("Connect")

    def closeEvent(self, event):
        self.connection_timer.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit(app.exec())
