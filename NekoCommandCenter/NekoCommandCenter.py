import sys
import json
import serial
import serial.tools.list_ports
from PySide6.QtCore import QTimer, Qt, QSize
from PySide6.Qt3DRender import Qt3DRender
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QWidget, QLabel,
    QGridLayout, QTextEdit, QSizePolicy, QLineEdit, QSpinBox, QDoubleSpinBox, QDialog,
    QFormLayout, QMenuBar, QMenu, QCheckBox
)
from PySide6.QtGui import QFont, QIcon, QColor, QPalette, QPainter, QBrush, QPixmap, QAction
from PySide6.Qt3DCore import Qt3DCore
from PySide6.Qt3DExtras import Qt3DExtras
from PySide6.QtGui import QVector3D, QMatrix4x4


class Object(Qt3DCore.QEntity):
    def __init__(self, parentEntity):
        super().__init__()

        # Define dimensions for a rectangular prism
        width, height, depth = 30, 15, 20  # X, Y, Z axes
        
        self.prismEntity = Qt3DCore.QEntity(parentEntity)

        # Plane positions and rotations
        planes_config = [
            # [x, y, z], [rotX, rotY, rotZ], face_name
            [[0, -height/2, 0], [0, 0, 180], "BOTTOM"],  # Bottom face
            [[0, height/2, 0], [0, 0, 0], "TOP"],        # Top face
            [[-width/2, 0, 0], [270, 90, 0], "LEFT"],    # Left face
            [[width/2, 0, 0], [-270, 90, 0], "RIGHT"],   # Right face
            [[0, 0, -depth/2], [270, 0, 0], "FRONT"],    # Front face
            [[0, 0, depth/2], [90, 0, 0], "BACK"]        # Back face
        ]
        
        # Define a single color for all faces
        self.object_color = QColor(85, 155, 255)
        text_color = QColor(10, 10, 10)  # Almost black text
        
        # Initialize arrays
        self.planeEntities = []
        self.planeMeshes = []
        self.planeTransforms = []
        self.materials = []
        self.textEntities = []

        # Build the planes
        for i, (translation, rotation, face_label) in enumerate(planes_config):
            # Create mesh
            mesh = Qt3DExtras.QPlaneMesh()
            
            # Set appropriate width and height for each face
            if i < 2:  # Top and bottom faces
                mesh.setWidth(width)
                mesh.setHeight(depth)
            elif i < 4:  # Left and right faces
                mesh.setWidth(depth)
                mesh.setHeight(height)
            else:  # Front and back faces
                mesh.setWidth(width)
                mesh.setHeight(height)
                
            # Create transform
            transform = Qt3DCore.QTransform()
            transform.setRotationX(rotation[0])
            transform.setRotationY(rotation[1])
            transform.setRotationZ(rotation[2])
            transform.setTranslation(QVector3D(*translation))
            
            # Create material
            material = Qt3DExtras.QPhongMaterial(self.prismEntity)
            material.setAmbient(self.object_color)
            material.setDiffuse(self.object_color)
            material.setShininess(200)
            material.setSpecular(QColor(80, 80, 80))
            
            # Create entity and add components
            entity = Qt3DCore.QEntity(self.prismEntity)
            entity.addComponent(mesh)
            entity.addComponent(transform)
            entity.addComponent(material)
            
            # Add text label
            text_entity = self.createTextEntity(
                parentEntity=entity,
                text=face_label,
                position=QVector3D(0, 0, 0.1),
                scale=0.15,
                color=text_color
            )
            
            # Store references
            self.planeEntities.append(entity)
            self.planeMeshes.append(mesh)
            self.planeTransforms.append(transform)
            self.materials.append(material)
            self.textEntities.append(text_entity)

        # Initial rotation
        self.prismTransform = Qt3DCore.QTransform()
        
    def createTextEntity(self, parentEntity, text, position, scale, color):
        """Create a text entity as a child of the given parent entity"""
        textEntity = Qt3DCore.QEntity(parentEntity)
        
        # Create text mesh
        textMesh = Qt3DExtras.QExtrudedTextMesh()
        textMesh.setText(text)
        textMesh.setDepth(0.2)
        textMesh.setFont(QFont("Arial", 12, QFont.Bold))
        
        # Create transform for positioning and scaling
        textTransform = Qt3DCore.QTransform()
        textTransform.setTranslation(position)
        textTransform.setScale(scale)
        
        # Create material
        textMaterial = Qt3DExtras.QPhongMaterial()
        textMaterial.setAmbient(color)
        textMaterial.setDiffuse(color)
        textMaterial.setShininess(0)
        
        # Assemble the entity
        textEntity.addComponent(textMesh)
        textEntity.addComponent(textTransform)
        textEntity.addComponent(textMaterial)
        
        return textEntity

    def applyQuaternion(self, q):
        """Apply a quaternion to rotate the prism"""
        # Convert quaternion to rotation matrix directly
        w, x, y, z = q
        rotation_matrix = QMatrix4x4(
            1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w, 0,
            2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w, 0,
            2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y, 0,
            0, 0, 0, 1
        )
        self.prismTransform.setMatrix(rotation_matrix)
        self.prismEntity.addComponent(self.prismTransform)


class Object3DWindow(Qt3DExtras.Qt3DWindow):
    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent
        
        # Serial communication properties
        self.serial = None
        self.device_name = "Not connected"
        self.flight_file_open = False  # Add this line
        
        # Setup 3D scene
        self.setupScene()
        
        # Setup timers
        self.setupTimers()
        
    def setupScene(self):
        # Root entity
        self.rootEntity = Qt3DCore.QEntity()
        self.setRootEntity(self.rootEntity)

        # Camera setup
        self.camera().lens().setPerspectiveProjection(45, 16/9, 0.1, 1000)
        self.camera().setPosition(QVector3D(30, 40, 50))
        self.camera().setUpVector(QVector3D(0, 1, 0))
        self.camera().setViewCenter(QVector3D(0, 0, 0))

        # Add lighting
        self.setupLighting()
        
        # Create 3D object
        self.object = Object(self.rootEntity)

    def setupLighting(self):
        # Ambient light
        self.ambientLight = Qt3DCore.QEntity(self.rootEntity)
        self.ambientLightComponent = Qt3DRender.QPointLight(self.ambientLight)
        self.ambientLightComponent.setColor(QColor(255, 255, 255))
        self.ambientLightComponent.setIntensity(0.8)
        self.ambientLightTransform = Qt3DCore.QTransform()
        self.ambientLightTransform.setTranslation(QVector3D(0, 40, 0))
        self.ambientLight.addComponent(self.ambientLightComponent)
        self.ambientLight.addComponent(self.ambientLightTransform)
        
        # Directional light
        self.directionalLight = Qt3DCore.QEntity(self.rootEntity)
        self.directionalLightComponent = Qt3DRender.QDirectionalLight(self.directionalLight)
        self.directionalLightComponent.setColor(QColor(200, 200, 200))
        self.directionalLightComponent.setIntensity(1.0)
        self.directionalLightComponent.setWorldDirection(QVector3D(1, -0.5, -0.5).normalized())
        self.directionalLight.addComponent(self.directionalLightComponent)

    def setupTimers(self):
        # Timer for object rotation
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.readSerialData)
        self.timer.start(20)

        # Timer for connection check
        self.connection_check_timer = QTimer(self)
        self.connection_check_timer.timeout.connect(self.check_connection)
        self.connection_check_timer.start(1000)

    def connectSerial(self):
        try:
            # Search for ports
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if port.pid == 0x8cb0 and port.vid in {0x732B, 0x5b99}:
                    self.serial = serial.Serial(port.device, 115200, timeout=1)
                    
                    # Set device name based on vendor ID
                    self.device_name = "Nekonet Receiver" if port.vid == 0x732B else "Nekonav"
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

    def readSerialData(self):
        """Reads serial data, parses sensor data and updates visualization."""
        try:
            if self.serial and self.serial.is_open and self.serial.in_waiting > 0:
                data = self.serial.readline().decode('utf-8').strip()

                if data.startswith("DATA:"):
                    try:
                        values = [float(num) for num in data[5:].split()]

                        if len(values) == 10:
                            accel_x, accel_y, accel_z = values[0:3]
                            q = values[3:7]
                            altitude = values[7]
                            time_diff = values[8]
                            state = int(values[9])

                            # Update object rotation
                            self.object.applyQuaternion(q)

                            # Update sensor display
                            if self.parent:
                                self.parent.sensor_display.update_values(
                                    accel_x, accel_y, accel_z, altitude, state
                                )
                    except ValueError as e:
                        self.logError(f"Data parsing error: {e}")
                elif data.startswith("FLIGHT:"):
                    # Handle flight data
                    self.logMessage(f"Receiving flight data...")
                    self.save_flight_data(data)
                elif data.startswith("LOG:"):
                    self.logMessage(data[4:])
                else:
                    self.logMessage(f"Unhandled Data: {data}")
                        
        except serial.SerialException as e:
            self.logError(f"Serial Exception: {e}")
            self.serial = None
            self.connectSerial()
        except Exception as e:
            self.logError(f"Error: {e}")


    def save_flight_data(self, data_line):
        """Saves raw flight data to a file, overwriting any previous data."""
        try:
            # Check if this is flight data
            if data_line.startswith("FLIGHT:"):
                # For the first flight data line - open a new file in write mode
                if not hasattr(self, 'flight_file_open') or not self.flight_file_open:
                    self.flight_file = open("flight_data.txt", 'w')  # 'w' mode overwrites existing file
                    self.flight_file_open = True
                    self.logMessage("Starting new flight data capture")
                
                # Write the raw data line to the file
                self.flight_file.write(f"{data_line}\n")
                self.flight_file.flush()  # Ensure data is written immediately
                
                # Log that we received data (might want to remove this in production to avoid log spam)
                self.logMessage("Flight data received")
                
            # If we're no longer receiving flight data, close the file
            elif hasattr(self, 'flight_file_open') and self.flight_file_open:
                self.flight_file.close()
                self.flight_file_open = False
                self.logMessage("Flight data capture complete. Saved to flight_data.txt")
                
        except Exception as e:
            self.logError(f"Error saving flight data: {e}")
            # Make sure to close the file on error
            if hasattr(self, 'flight_file_open') and self.flight_file_open:
                self.flight_file.close()
                self.flight_file_open = False


    def logMessage(self, message):
        if self.parent and self.parent.serial_log:
            self.parent.serial_log.append(message)

    def logError(self, error):
        if self.parent and self.parent.serial_log:
            self.parent.serial_log.append(error)

    def sendData(self, data):
        """Sends data to the serial port and logs it."""
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(f"{data}\n".encode('utf-8'))
                self.logMessage(f"Sent: {data}")
            except Exception as e:
                self.logError(f"Error sending data: {e}")
                self.serial = None

    def check_connection(self):
        if self.serial and not self.serial.is_open:
            print("Serial port closed unexpectedly.")
            self.disconnectSerial()
            # Try to reconnect
            if self.parent:
                self.parent.connect_to_device()


class SettingsDialog(QDialog):
    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.config = config
        self.setWindowTitle("Altimeter Settings")
        
        self.setupUI()

    def setupUI(self):
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

        # Primary Delay (short)
        self.Primary_delay_input = QSpinBox()
        self.Primary_delay_input.setRange(0, 32767)
        self.Primary_delay_input.setValue(int(self.config.get("Primary Delay [s]", 0)))
        self.layout.addRow("Primary Delay [s]:", self.Primary_delay_input)

        # Secondary Delay (short)
        self.Secondary_delay_input = QSpinBox()
        self.Secondary_delay_input.setRange(0, 32767)
        self.Secondary_delay_input.setValue(int(self.config.get("Secondary Delay [s]", 1)))
        self.layout.addRow("Secondary Delay [s]:", self.Secondary_delay_input)

        # Backup State (short)
        self.backup_state_input = QSpinBox()
        self.backup_state_input.setRange(0, 32767)
        self.backup_state_input.setValue(int(self.config.get("Backup State", 1)))
        self.layout.addRow("Backup State:", self.backup_state_input)

        # Main Altitude (long)
        self.main_altitude_input = QSpinBox()
        self.main_altitude_input.setRange(0, 2147483647)
        self.main_altitude_input.setValue(int(self.config.get("Main Altitude", 1500)))
        self.layout.addRow("Main Altitude:", self.main_altitude_input)
        
        # Auxiliary Function (uint8_t)
        self.auxiliary_function_input = QSpinBox()
        self.auxiliary_function_input.setRange(0, 255)
        self.auxiliary_function_input.setValue(self.config.get("Auxiliary Function", 0))
        self.layout.addRow("Auxiliary Function:", self.auxiliary_function_input)

        # Save button
        self.save_button = QPushButton("Save")
        self.save_button.clicked.connect(self.save_settings)
        self.layout.addWidget(self.save_button)

    def save_settings(self):
        self.config["Callsign"] = self.callsign_input.text()
        self.config["Frequency Mhz"] = f"{self.frequency_input.value():.3f}"
        self.config["Primary Delay [s]"] = self.Primary_delay_input.value()
        self.config["Secondary Delay [s]"] = self.Secondary_delay_input.value()
        self.config["Backup State"] = self.backup_state_input.value()
        self.config["Main Altitude"] = self.main_altitude_input.value()
        self.config["Auxiliary Function"] = self.auxiliary_function_input.value()

        with open("config.json", "w") as f:
            json.dump(self.config, f, indent=4)

        self.accept()


class SensorDataDisplay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUI()

    def setupUI(self):
        self.layout = QHBoxLayout()
        self.setLayout(self.layout)
        
        # Add initial stretch for centering
        self.layout.addStretch()
        
        # Create displays for each sensor value
        self.labels = {
            "accel_x": QLabel("AccelX: 0.00"),
            "accel_y": QLabel("AccelY: 0.00"),
            "accel_z": QLabel("AccelZ: 0.00"),
            "altitude": QLabel("Alt: 0.00m"),
            "state": QLabel("State: 0")
        }
        
        # Add labels to layout with consistent formatting
        for label in self.labels.values():
            label.setMinimumWidth(120)
            label.setAlignment(Qt.AlignCenter)
            self.layout.addWidget(label)
        
        # Add ending stretch for centering
        self.layout.addStretch()
        self.layout.setContentsMargins(0, 0, 0, 0)

    def update_values(self, accel_x, accel_y, accel_z, altitude, state):
        self.labels["accel_x"].setText(f"AccelX: {accel_x:.2f}")
        self.labels["accel_y"].setText(f"AccelY: {accel_y:.2f}")
        self.labels["accel_z"].setText(f"AccelZ: {accel_z:.2f}")
        self.labels["altitude"].setText(f"Alt: {altitude:.1f}m")
        self.labels["state"].setText(f"State: {state}")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Application")
        self.resize(800, 600)
        
        # Init config
        self.load_config()
        
        # Setup UI
        self.setupUI()
        
        # Apply initial theme
        self.apply_theme(self.config.get("Theme", "Light Mode"))
        
        self.show()

    def setupUI(self):
        # Create 3D Window
        self.window3D = Object3DWindow(parent=self)
        self.widget3D = self.createWindowContainer(self.window3D)
        self.widget3D.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.widget3D.setVisible(False)
        
        # Main layout
        self.central_widget = QWidget()
        self.grid_layout = QGridLayout(self.central_widget)
        self.setCentralWidget(self.central_widget)
        
        # Create UI components
        self.createConnectionBar()
        self.createSensorDisplay()
        self.createMainDisplayArea()
        self.createBottomButtons()
        self.createMenuBar()
        
        # Layout everything
        self.setupLayout()

    def createConnectionBar(self):
        # Top connection bar
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.connect_to_device)
        self.device_label = QLabel("Device: Not connected")
        
        self.top_layout = QHBoxLayout()
        self.top_layout.addWidget(self.btn_connect)
        self.top_layout.addWidget(self.device_label)
        self.top_layout.addStretch()

    def createSensorDisplay(self):
        # Sensor data display
        self.sensor_display = SensorDataDisplay()

    def createMainDisplayArea(self):
        # Logo placeholder
        self.logo_placeholder = QLabel(alignment=Qt.AlignCenter)
        self.logo_placeholder.setPixmap(QPixmap("Logo.png"))
        self.logo_placeholder.setScaledContents(True)
        self.logo_placeholder.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Serial log
        self.serial_log = QTextEdit()
        self.serial_log.setReadOnly(True)
        self.serial_log.setPlaceholderText("Serial Communication Log")
        self.serial_log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.serial_log.setVisible(False)

    def createBottomButtons(self):
        # Button commands dictionary
        self.cmd_buttons = {
            "Read Flight": 160,
            "Delete Flight": 52,
            "Program Device": 150,
            "Calibrate Altimeter": 255
        }
        
        self.buttons = {}
        self.buttons_layout = QHBoxLayout()
        self.buttons_layout.addStretch()
        
        # Create buttons from dictionary
        for label, cmd in self.cmd_buttons.items():
            btn = QPushButton(label)
            if label == "Program Device":
                btn.clicked.connect(self.send_program_command)
            else:
                btn.clicked.connect(lambda checked, cmd=cmd: self.window3D.sendData(cmd))
            self.buttons[label] = btn
            self.buttons_layout.addWidget(btn)
            
        self.buttons_layout.addStretch()

    def createMenuBar(self):
        # Menu bar
        self.menu_bar = QMenuBar()
        self.setMenuBar(self.menu_bar)
        
        # Settings menu
        self.settings_menu = QMenu("Settings", self)
        self.menu_bar.addMenu(self.settings_menu)
        
        # Menu actions
        menu_actions = {
            "Altimeter Settings": self.open_settings_dialog,
            "Toggle Theme": self.toggle_theme,
            "Toggle Object Display": self.toggle_object_display,
            "Toggle Serial Display": self.toggle_serial_display
        }
        
        for label, func in menu_actions.items():
            action = QAction(label, self)
            action.triggered.connect(func)
            self.settings_menu.addAction(action)

    def setupLayout(self):
        # Add everything to grid layout
        current_row = 0
        
        # Top connection bar
        self.grid_layout.addLayout(self.top_layout, current_row, 0, 1, 5)
        
        # Sensor display
        current_row += 1
        self.grid_layout.addWidget(self.sensor_display, current_row, 0, 1, 5)
        
        # Main display area
        current_row += 1
        self.grid_layout.addWidget(self.logo_placeholder, current_row, 0, 1, 5)
        self.grid_layout.addWidget(self.widget3D, current_row, 0, 1, 5)
        self.grid_layout.addWidget(self.serial_log, current_row, 0, 1, 5)
        
        # Bottom buttons
        current_row += 1
        self.grid_layout.addLayout(self.buttons_layout, current_row, 0, 1, 5)

    def load_config(self):
        try:
            with open("config.json", "r") as f:
                self.config = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            self.config = {
                "Callsign": "",
                "Frequency Mhz": "0.000",
                "Primary Delay [s]": "0",
                "Secondary Delay [s]": "1",
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
        """Apply the selected theme (Light Mode or Dark Mode) to the application"""
        if theme == "Light Mode":
            # Seafoam Green theme colors
            stylesheet = """
                QMainWindow, QDialog { 
                    background-color: #e8f4f2; 
                    color: #2c3e50; 
                }
                
                QWidget { 
                    background-color: #e8f4f2; 
                    color: #2c3e50;
                }
                
                QLabel { 
                    color: #2c3e50; 
                    background-color: transparent;
                }
                
                QPushButton { 
                    background-color: #a8d5cb; 
                    color: #2c3e50; 
                    border: none;
                    border-radius: 4px;
                    padding: 5px 10px;
                }
                
                QPushButton:hover { 
                    background-color: #8fcbbf; 
                }
                
                QPushButton:pressed { 
                    background-color: #70b2a3; 
                    color: #ffffff; 
                }
                
                QTextEdit, QLineEdit, QSpinBox, QDoubleSpinBox { 
                    background-color: #f5faf8; 
                    color: #2c3e50; 
                    border: 1px solid #b8d8d0; 
                    border-radius: 3px;
                    padding: 2px;
                }
                
                QMenuBar { 
                    background-color: #e8f4f2; 
                    color: #2c3e50; 
                }
                
                QMenuBar::item:selected { 
                    background-color: #a8d5cb; 
                    color: #2c3e50; 
                }
                
                QMenu { 
                    background-color: #e8f4f2; 
                    color: #2c3e50; 
                    border: 1px solid #b8d8d0; 
                }
                
                QMenu::item:selected { 
                    background-color: #a8d5cb; 
                    color: #2c3e50; 
                }
            """
        else:  # Dark Mode
            # Dark theme colors
            stylesheet = """
                QMainWindow, QDialog { 
                    background-color: #263238; 
                    color: #ecf0f1; 
                }
                
                QWidget { 
                    background-color: #263238; 
                    color: #ecf0f1;
                }
                
                QLabel { 
                    color: #ecf0f1; 
                    background-color: transparent;
                }
                
                QPushButton { 
                    background-color: #375a53; 
                    color: #ecf0f1; 
                    border: none;
                    border-radius: 4px;
                    padding: 5px 10px;
                }
                
                QPushButton:hover { 
                    background-color: #437b70; 
                }
                
                QPushButton:pressed { 
                    background-color: #57a99a; 
                    color: #ffffff; 
                }
                
                QTextEdit, QLineEdit, QSpinBox, QDoubleSpinBox { 
                    background-color: #2f3e46; 
                    color: #ecf0f1; 
                    border: 1px solid #456c60; 
                    border-radius: 3px;
                    padding: 2px;
                }
                
                QMenuBar { 
                    background-color: #263238; 
                    color: #ecf0f1; 
                }
                
                QMenuBar::item:selected { 
                    background-color: #57a99a; 
                    color: #ecf0f1; 
                }
                
                QMenu { 
                    background-color: #263238; 
                    color: #ecf0f1; 
                    border: 1px solid #456c60; 
                }
                
                QMenu::item:selected { 
                    background-color: #57a99a; 
                    color: #ecf0f1; 
                }
            """
        
        # Apply the stylesheet
        self.setStyleSheet(stylesheet)

    def toggle_object_display(self):
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

    def send_program_command(self):
        """Send the program command (150) followed by configuration data"""
        if self.window3D.serial and self.window3D.serial.is_open:
            # First send the command code
            self.window3D.sendData(150)
            
            # Wait a short time for the Arduino to process the command
            QTimer.singleShot(500, self.send_config_data)
        else:
            self.window3D.logError("Cannot program device: No serial connection")
            
    def send_config_data(self):
        """Send configuration data to the device"""
        if self.window3D.serial and self.window3D.serial.is_open:
            try:
                # Format config data as a pipe-delimited string, ensuring we use correct types
                callsign = self.config.get('Callsign', '')
                frequency = float(self.config.get('Frequency Mhz', '0.000'))
                main_altitude = int(self.config.get('Main Altitude', 1500))
                primary_delay = int(self.config.get('Primary Delay [s]', 0))
                secondary_delay = int(self.config.get('Secondary Delay [s]', 1))
                backup_state = int(self.config.get('Backup State', 1))
                auxiliary_function = int(self.config.get('Auxiliary Function', 0))
                
                config_string = (
                    f"{callsign}|"
                    f"{frequency:.3f}|"
                    f"{main_altitude}|"
                    f"{primary_delay}|"
                    f"{secondary_delay}|"
                    f"{backup_state}|"
                    f"{auxiliary_function}"
                )
                
                # Send the configuration string
                self.window3D.serial.write(f"{config_string}\n".encode('utf-8'))
                self.window3D.logMessage(f"Sent configuration: {config_string}")
            except Exception as e:
                self.window3D.logError(f"Error sending configuration data: {e}")
        else:
            self.window3D.logError("Lost connection while programming device")
        
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = MainWindow()
    sys.exit(app.exec())