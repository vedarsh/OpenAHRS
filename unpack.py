import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import threading

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/cu.usbmodem338F397533351' 
BAUD_RATE = 115200

# Packet: Start(1) + 3*Accel(4) + 3*Gyro(4) + 3*Mag(4) + 4*Fused(4) + End(1)
# Total: 1 + 12 + 12 + 12 + 16 + 1 = 54 bytes
PACKET_FMT = '<BfffffffffffffB' 
PACKET_SIZE = struct.calcsize(PACKET_FMT)

# Global Data
current_data = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
running = True

def read_serial_thread():
    """ Runs in background to keep buffer empty """
    global current_data, running
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}")
        ser.reset_input_buffer()

        while running:
            # Sync
            if ser.read(1) != b'\xAA': continue

            # Read Body
            data = ser.read(PACKET_SIZE - 1)
            if len(data) != (PACKET_SIZE - 1): continue
            
            # Unpack
            unpacked = struct.unpack('<fffffffffffffB', data)
            
            # Extract Fused Data (Last 4 floats before footer)
            # Indices: 0-2 (Acc), 3-5 (Gyro), 6-8 (Mag), 9 (Roll), 10 (Pitch), 11 (Yaw), 12 (Head)
            roll  = unpacked[9]
            pitch = unpacked[10]
            yaw   = unpacked[11]
            footer = unpacked[13]

            if footer == 0x55:
                # Store
                current_data['roll'] = roll
                current_data['pitch'] = pitch
                current_data['yaw'] = yaw

    except Exception as e:
        print(f"Serial Error: {e}")
        running = False
    finally:
        if 'ser' in locals() and ser.is_open: ser.close()

def get_rotation_matrix(roll, pitch, yaw):
    # Convert to Radians
    phi = np.radians(roll)
    theta = np.radians(pitch)
    psi = np.radians(yaw)

    # Standard Rotation Matrices
    Rx = np.array([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
    Ry = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
    Rz = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]])

    return Rz @ Ry @ Rx

def visualize():
    global running
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(bottom=0.15) 

    # Basis Vectors
    basis = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) # X, Y, Z

    print("Plotting... Close window to stop.")

    try:
        while running:
            plt.cla() 
            
            # --- SWAP PITCH AND ROLL HERE ---
            # Original: r=roll, p=pitch
            # Swapped: r=pitch, p=roll (or however you need them mapped)
            r = current_data['pitch'] # Swapped
            p = current_data['roll']  # Swapped
            y = current_data['yaw']

            # Rotate
            R = get_rotation_matrix(r, p, y)
            new_basis = (R @ basis.T).T

            # Plot Vectors
            # X (Red), Y (Green), Z (Blue)
            colors = ['r', 'g', 'b']
            for i in range(3):
                ax.quiver(0, 0, 0, new_basis[i,0], new_basis[i,1], new_basis[i,2], 
                          color=colors[i], length=1.0, linewidth=3)

            # Format
            ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([-1, 1])
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
            ax.set_title(f"AHRS Orientation (Swapped R/P)\nRoll(mapped to Pitch): {r:.1f}°\nPitch(mapped to Roll): {p:.1f}°\nYaw: {y:.1f}°")
            
            plt.pause(0.05) 
            if not plt.fignum_exists(fig.number): running = False

    except KeyboardInterrupt:
        running = False

if __name__ == "__main__":
    t = threading.Thread(target=read_serial_thread)
    t.daemon = True
    t.start()
    visualize()
