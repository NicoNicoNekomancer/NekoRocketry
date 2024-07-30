from ttkthemes import ThemedTk
import serial
import serial.tools.list_ports
import struct
import threading
import json
import os
from tkinter import ttk  # Needed for the combo box
import tkinter as tk
import time

def read_from_serial(text):
    global ser
    while True:
        try:
            if ser is not None and ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                text.insert(tk.END, f"Received line: {line}\n")  # Display the line in the text widget
                text.see(tk.END)  # Auto-scroll to the end
        except serial.SerialException:
            text.insert(tk.END, "Device disconnected.\n")
            ser = None

def send_data(ser, text, entries, pressureTransducer, altitudeUnit):
    if ser is None:
        text.insert(tk.END, "No device connected. Cannot send data.\n")
        return
    text.insert(tk.END, "Button1 pressed. Sending data...\n")
    ser.write(b'153\n')  # Send 153 to enter READ state
    text.insert(tk.END, "Data sent.\nWaiting for the handshake message...\n")
    # Send the entry text as bytes
    data = {}
    for name, entry in entries.items():
        value = entry.get()
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
            if altitudeUnit.get() == 'Feet':
                value = round(value * 0.3048)  # Convert feet to meters and round it
        data[name] = value
    # Get the value from the pressureTransducer combo box
    pressureTransducer_value = pressureTransducer.get()
    data['Pressure Transducer'] = pressureTransducer_value == 'Yes'
    # Get the value from the altitudeUnit combo box
    data['Altitude Unit'] = altitudeUnit.get()
    try:
        message = struct.pack('<6s 4s h h l h ?', *[data[name] for name in entries] + [data['Pressure Transducer']])
        ser.write(message)
        text.insert(tk.END, f"{message} sent as bytes.\n")
    except struct.error as e:
        text.insert(tk.END, f"Error packing data: {e}\n")
    # Save the entry text to a json file
    with open('config.json', 'w') as f:
        json.dump({name: entry.get() for name, entry in entries.items()}, f)

def find_device(vid, pids, text):
    global ser
    while True:
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if p.vid == vid and p.pid in pids and "USB Serial Device" in p.description:
                if p.pid == 0x8CB0:
                    device_name = "Nekonet Receiver"
                elif p.pid == 0x5B99:
                    device_name = "NekoNav"
                if ser is None or ser.port != p.device:
                    if ser is not None:
                        ser.close()  # Close the previous serial connection
                    ser = serial.Serial(p.device, 115200)  # Adjust baud rate as needed
                    text.insert(tk.END, f"Connected to {device_name} on port {p.device}\n")
        time.sleep(1)  # Wait for a second before checking again

# VID and PIDs for the devices
vid = 0x732B
pids = [0x8CB0, 0x5B99]

ser = None

root = ThemedTk(theme="equilux")  # Create a themed root window with "equilux" as the default theme

mainframe = ttk.Frame(root)  # Create a main frame to hold all widgets
mainframe.pack(fill='both', expand=True)  # Make the main frame fill the whole window

text = tk.Text(mainframe, bg='white', fg='black')  # Create a text widget with manually set colors
text.pack()

frame = ttk.Frame(mainframe)  # Create a new frame inside the main frame
frame.pack()

# Create an entry widget for each data type
entry_names = ['Callsign', 'Frequency Mhz', 'Drogue Delay [s]', 'Main Delay [s]', 'Backup State']
entries = {name: ttk.Entry(frame) for name in entry_names}
for i, (name, entry) in enumerate(entries.items()):
    ttk.Label(frame, text=name).grid(row=i, column=0)  # Place the label in the grid
    entry.grid(row=i, column=1)  # Place the entry in the grid

# Create an entry and a combo box for 'Main Altitude' and 'Altitude Unit'
ttk.Label(frame, text='Main Altitude').grid(row=len(entries), column=0)
entries['Main Altitude'] = ttk.Entry(frame)
entries['Main Altitude'].grid(row=len(entries), column=1)
altitudeUnit = ttk.Combobox(frame, values=['Feet', 'Meters'], width=6)
altitudeUnit.grid(row=len(entries), column=2)

# Create a combo box for 'Pressure Transducer'
ttk.Label(frame, text='Pressure Transducer').grid(row=len(entries)+1, column=0)
pressureTransducer = ttk.Combobox(frame, values=['Yes', 'No'], width=3)
pressureTransducer.grid(row=len(entries)+1, column=1)

# Create a combo box for 'Theme'
ttk.Label(frame, text='Theme').grid(row=0, column=3)
theme = ttk.Combobox(frame, values=list(root.get_themes()), width=6)
theme.grid(row=0, column=4)

def change_theme(event):
    new_theme = theme.get()
    root.set_theme(new_theme)
    if new_theme == 'arc':  # Replace 'arc' with the name of your theme
        text.configure(bg='your_background_color', fg='your_foreground_color')
    # Add more elif statements for other themes

theme.bind('<<ComboboxSelected>>', change_theme)

# Load the last config from the json file if it exists
if os.path.exists('config.json'):
    with open('config.json', 'r') as f:
        content = f.read().strip()
        if content:  # Check if the file is not empty
            last_config = json.loads(content)
            for name, entry in entries.items():
                if name in last_config:
                    entry.insert(0, last_config[name])
            if 'Pressure Transducer' in last_config:
                pressureTransducer.set(last_config['Pressure Transducer'])
            if 'Altitude Unit' in last_config:
                altitudeUnit.set(last_config['Altitude Unit'])
            if 'Theme' in last_config:
                theme.set(last_config['Theme'])
                root.set_theme(last_config['Theme'])

button1 = ttk.Button(frame, text="Program", command=lambda: send_data(ser, text, entries, pressureTransducer, altitudeUnit))
button1.grid(row=len(entries)+2, column=0, columnspan=5)

# Start a new thread to find the device
threading.Thread(target=find_device, args=(vid, pids, text)).start()

# Start a new thread to read from the serial port
threading.Thread(target=read_from_serial, args=(text,)).start()

root.mainloop()
