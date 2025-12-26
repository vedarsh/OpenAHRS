import sys
import struct
import numpy as np
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QComboBox, QPushButton, QLabel
from PyQt5.QtCore import QTimer, pyqtSignal, QThread
import pyqtgraph as pg
import pyqtgraph.opengl as gl

# ------------------------------------------------------------------
# CONFIGURATION
# ------------------------------------------------------------------
BAUD_RATE = 115200

# PACKET DEFINITION
# Previous: 54 bytes. New: 54 + 1 (RSSI) = 55 bytes.
# Format: Start(1) + 13*Floats(52) + End(1) + RSSI(1, signed char)
STRUCT_FORMAT = '<BfffffffffffffBb' 
EXPECTED_SIZE = 55

# ------------------------------------------------------------------
# SERIAL WORKER THREAD
# ------------------------------------------------------------------
class SerialWorker(QThread):
    data_received = pyqtSignal(list)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, BAUD_RATE, timeout=0.1)
            print(f"Connected to {self.port}")
            
            while self.running:
                if self.ser.in_waiting >= EXPECTED_SIZE:
                    
                    # Sync to Start Byte
                    while True:
                        if self.ser.read(1) == b'\xAA':
                            remaining = self.ser.read(EXPECTED_SIZE - 1)
                            if len(remaining) == EXPECTED_SIZE - 1:
                                full_packet = b'\xAA' + remaining
                                self.parse_packet(full_packet)
                            break
                        if self.ser.in_waiting < EXPECTED_SIZE:
                            break
                            
        except Exception as e:
            print(f"Serial Error: {e}")
        finally:
            if self.ser: self.ser.close()

    def parse_packet(self, raw_data):
        try:
            unpacked = struct.unpack(STRUCT_FORMAT, raw_data)
            
            # unpacked[0] = Start (0xAA)
            # unpacked[14] = End (0x55)
            # unpacked[15] = RSSI (int8)
            
            if unpacked[14] == 0x55:
                # Extract Floats (indices 1 to 13)
                sensor_data = list(unpacked[1:14])
                # Extract RSSI
                rssi = unpacked[15]
                
                # Append RSSI to data list for main thread
                sensor_data.append(rssi)
                self.data_received.emit(sensor_data)
        except struct.error:
            pass 

    def stop(self):
        self.running = False
        self.wait()

# ------------------------------------------------------------------
# MAIN VISUALIZATION WINDOW
# ------------------------------------------------------------------
class AHRSVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32/ESP32 AHRS Dashboard")
        self.resize(1200, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # ---------------- LEFT: 3D ORIENTATION ----------------
        self.gl_view = gl.GLViewWidget()
        self.gl_view.setCameraPosition(distance=20)
        self.gl_view.setBackgroundColor('k')
        
        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)
        self.gl_view.addItem(grid)

        self.cube = gl.GLBoxItem(size=QVector3D(3, 1, 5), color=(200, 200, 200, 255))
        self.cube.translate(-1.5, -0.5, -2.5) 
        self.axis = gl.GLAxisItem()
        self.axis.setSize(10, 10, 10)
        
        self.gl_view.addItem(self.cube)
        self.gl_view.addItem(self.axis)

        main_layout.addWidget(self.gl_view, stretch=2)

        # ---------------- RIGHT: SENSOR PLOTS ----------------
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        main_layout.addWidget(right_panel, stretch=3)

        # Connection Controls
        conn_layout = QHBoxLayout()
        self.combo_ports = QComboBox()
        self.btn_refresh = QPushButton("Refresh Ports")
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setCheckable(True)
        
        # --- NEW: RSSI Label ---
        self.lbl_rssi = QLabel("Signal: -- dBm")
        self.lbl_rssi.setStyleSheet("font-size: 14px; font-weight: bold; padding: 5px; color: #444;")
        
        conn_layout.addWidget(self.combo_ports)
        conn_layout.addWidget(self.btn_refresh)
        conn_layout.addWidget(self.btn_connect)
        conn_layout.addSpacing(20)
        conn_layout.addWidget(self.lbl_rssi) # Add label to layout
        
        right_layout.addLayout(conn_layout)

        # Initialize Graphs
        self.plot_accel = self.create_plot("Accelerometer", "m/s²", ['r', 'g', 'b'])
        self.plot_gyro = self.create_plot("Gyroscope", "deg/s", ['r', 'g', 'b'])
        self.plot_mag = self.create_plot("Magnetometer", "uT", ['r', 'g', 'b'])
        
        right_layout.addWidget(self.plot_accel)
        right_layout.addWidget(self.plot_gyro)
        right_layout.addWidget(self.plot_mag)

        # Data Buffers
        self.buffer_size = 200
        self.data_accel = np.zeros((3, self.buffer_size))
        self.data_gyro  = np.zeros((3, self.buffer_size))
        self.data_mag   = np.zeros((3, self.buffer_size))

        self.btn_refresh.clicked.connect(self.scan_ports)
        self.btn_connect.clicked.connect(self.toggle_connection)
        
        self.scan_ports()
        self.worker = None

    def create_plot(self, title, units, colors):
        p = pg.PlotWidget(title=title)
        p.showGrid(x=True, y=True)
        p.addLegend()
        p.setLabel('left', units)
        p.plot_lines = []
        labels = ['X', 'Y', 'Z']
        for i, c in enumerate(colors):
            line = p.plot(pen=c, name=labels[i])
            p.plot_lines.append(line)
        return p

    def scan_ports(self):
        self.combo_ports.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_ports.addItem(p.device)

    def toggle_connection(self):
        if self.btn_connect.isChecked():
            port = self.combo_ports.currentText()
            if not port: return
            
            self.worker = SerialWorker(port)
            self.worker.data_received.connect(self.update_data)
            self.worker.start()
            self.btn_connect.setText("Disconnect")
        else:
            if self.worker:
                self.worker.stop()
            self.btn_connect.setText("Connect")

    def update_data(self, data):
        # Unpack Data
        # 0-2: Accel, 3-5: Gyro, 6-8: Mag, 9-11: RPY, 12: Head, 13: RSSI
        ax, ay, az = data[0], data[1], data[2]
        gx, gy, gz = data[3], data[4], data[5]
        mx, my, mz = data[6], data[7], data[8]
        roll, pitch, yaw = data[9], data[10], data[11]
        rssi = data[13]

        # 1. Update RSSI Label
        self.lbl_rssi.setText(f"Signal: {rssi} dBm")
        
        # Color code RSSI
        if rssi > -50: self.lbl_rssi.setStyleSheet("color: green; font-weight: bold;")
        elif rssi > -70: self.lbl_rssi.setStyleSheet("color: orange; font-weight: bold;")
        else: self.lbl_rssi.setStyleSheet("color: red; font-weight: bold;")

        # 2. Update 3D Cube
        self.cube.resetTransform()
        self.cube.rotate(yaw,   0, 0, 1) 
        self.cube.rotate(pitch, 0, 1, 0) 
        self.cube.rotate(roll,  1, 0, 0) 
        self.cube.translate(-1.5, -0.5, -2.5) 

        # 3. Update Plots
        self.data_accel = np.roll(self.data_accel, -1, axis=1)
        self.data_gyro = np.roll(self.data_gyro, -1, axis=1)
        self.data_mag = np.roll(self.data_mag, -1, axis=1)

        self.data_accel[:, -1] = [ax, ay, az]
        self.data_gyro[:, -1]  = [gx, gy, gz]
        self.data_mag[:, -1]   = [mx, my, mz]

        self.update_plot_curves(self.plot_accel, self.data_accel)
        self.update_plot_curves(self.plot_gyro, self.data_gyro)
        self.update_plot_curves(self.plot_mag, self.data_mag)

    def update_plot_curves(self, plot_widget, data):
        for i in range(3):
            plot_widget.plot_lines[i].setData(data[i])

if __name__ == '__main__':
    from PyQt5.QtGui import QVector3D 
    app = QApplication(sys.argv)
    window = AHRSVisualizer()
    window.show()
    sys.exit(app.exec_())
